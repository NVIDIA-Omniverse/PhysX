//! Auto-bond authoring helpers built on Blast's pre-fractured bond generator.

use std::error::Error;
use std::fmt;
use std::ptr;
use std::slice;

use crate::ffi;
use crate::types::{ScenarioBond, ScenarioCollider, ScenarioDesc, ScenarioNode, Vec3};

/// Bond-generation mode used by Blast authoring.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BondingMode {
    /// Exact triangle-face matching. Adjacent pieces must share faces exactly.
    Exact,
    /// Approximate bonding with a gap tolerance in meters.
    Average { max_separation: f32 },
}

/// Forward-compatible options for bond generation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BondingOptions {
    pub mode: BondingMode,
}

impl Default for BondingOptions {
    fn default() -> Self {
        Self {
            mode: BondingMode::Exact,
        }
    }
}

/// Triangle soup for one fracture piece.
///
/// Triangles must already be expressed in the same coordinate space as the
/// target scenario. Each group of three vertices forms one triangle.
#[derive(Clone, Debug, Default)]
pub struct TriangleChunk {
    pub triangles: Vec<Vec3>,
    pub bondable: bool,
}

/// Convenience input for building a full `ScenarioDesc` from piece meshes.
#[derive(Clone, Debug)]
pub struct ScenarioPiece {
    pub node: ScenarioNode,
    pub triangles: Vec<Vec3>,
    pub bondable: bool,
    pub node_size: Option<Vec3>,
    pub collider_shape: Option<ScenarioCollider>,
}

/// Errors returned by the authoring helpers.
#[derive(Clone, Debug, PartialEq)]
pub enum AuthoringError {
    InvalidTriangleVertexCount {
        chunk_index: usize,
        vertex_count: usize,
    },
    InvalidMaxSeparation(f32),
    TooManyChunks(usize),
    TooManyTriangles(usize),
    NativeFailure(&'static str),
}

impl fmt::Display for AuthoringError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidTriangleVertexCount {
                chunk_index,
                vertex_count,
            } => write!(
                f,
                "chunk {} has {} vertices; triangle chunks require a multiple of 3 vertices",
                chunk_index, vertex_count
            ),
            Self::InvalidMaxSeparation(value) => write!(
                f,
                "average bonding requires max_separation > 0, got {}",
                value
            ),
            Self::TooManyChunks(count) => {
                write!(
                    f,
                    "authoring received {} chunks, which exceeds u32::MAX",
                    count
                )
            }
            Self::TooManyTriangles(count) => write!(
                f,
                "authoring received {} triangles, which exceeds u32::MAX",
                count
            ),
            Self::NativeFailure(message) => f.write_str(message),
        }
    }
}

impl Error for AuthoringError {}

/// Generate bonds directly from triangle chunks.
pub fn create_bonds_from_triangles(
    chunks: &[TriangleChunk],
    options: &BondingOptions,
) -> Result<Vec<ScenarioBond>, AuthoringError> {
    if chunks.is_empty() {
        return Ok(Vec::new());
    }

    let mesh_count =
        u32::try_from(chunks.len()).map_err(|_| AuthoringError::TooManyChunks(chunks.len()))?;
    let (bond_mode, max_separation) = resolve_mode(options.mode)?;

    let mut offsets = Vec::with_capacity(chunks.len() + 1);
    offsets.push(0u32);

    let total_vertices: usize = chunks.iter().map(|chunk| chunk.triangles.len()).sum();
    let total_triangles = total_vertices / 3;
    let total_floats = total_vertices
        .checked_mul(3)
        .ok_or(AuthoringError::TooManyTriangles(total_triangles))?;
    let triangle_float_count = u32::try_from(total_floats)
        .map_err(|_| AuthoringError::TooManyTriangles(total_triangles))?;

    let mut flattened = Vec::with_capacity(total_floats);
    let mut bondable_flags = Vec::with_capacity(chunks.len());

    for (index, chunk) in chunks.iter().enumerate() {
        if chunk.triangles.len() % 3 != 0 {
            return Err(AuthoringError::InvalidTriangleVertexCount {
                chunk_index: index,
                vertex_count: chunk.triangles.len(),
            });
        }

        let triangle_count = u32::try_from(chunk.triangles.len() / 3)
            .map_err(|_| AuthoringError::TooManyTriangles(chunk.triangles.len() / 3))?;
        let next = offsets[index]
            .checked_add(triangle_count)
            .ok_or(AuthoringError::TooManyTriangles(total_triangles))?;
        offsets.push(next);

        for vertex in &chunk.triangles {
            flattened.push(vertex.x);
            flattened.push(vertex.y);
            flattened.push(vertex.z);
        }

        bondable_flags.push(if chunk.bondable { 1u8 } else { 0u8 });
    }

    let mut out_ptr: *mut ffi::FfiExtStressBondDesc = ptr::null_mut();
    let bond_count = unsafe {
        ffi::authoring_bonds_from_prefractured_triangles(
            mesh_count,
            offsets.as_ptr(),
            flattened.as_ptr(),
            triangle_float_count,
            bondable_flags.as_ptr(),
            bond_mode,
            max_separation,
            &mut out_ptr,
        )
    };

    if bond_count == 0 {
        if !out_ptr.is_null() {
            unsafe { ffi::authoring_free(out_ptr.cast()) };
        }
        return Ok(Vec::new());
    }

    if out_ptr.is_null() {
        return Err(AuthoringError::NativeFailure(
            "authoring bridge returned bonds without an output buffer",
        ));
    }

    let raw = unsafe { slice::from_raw_parts(out_ptr, bond_count as usize) };
    let bonds = raw
        .iter()
        .map(|bond| ScenarioBond {
            node0: bond.node0,
            node1: bond.node1,
            centroid: bond.centroid,
            normal: bond.normal,
            area: bond.area.max(1.0e-8),
        })
        .collect();

    unsafe { ffi::authoring_free(out_ptr.cast()) };
    Ok(bonds)
}

/// Build a complete `ScenarioDesc` from meshes and per-piece node metadata.
///
/// This convenience helper keeps the API close to the desired consumer flow:
/// "here are all the pieces, auto-bond them."
pub fn build_scenario_from_pieces(
    pieces: &[ScenarioPiece],
    options: &BondingOptions,
) -> Result<ScenarioDesc, AuthoringError> {
    if pieces.is_empty() {
        return Ok(ScenarioDesc::default());
    }

    let chunks: Vec<TriangleChunk> = pieces
        .iter()
        .map(|piece| TriangleChunk {
            triangles: piece.triangles.clone(),
            bondable: piece.bondable,
        })
        .collect();

    let bonds = create_bonds_from_triangles(&chunks, options)?;
    let nodes = pieces.iter().map(|piece| piece.node).collect();
    let node_sizes = pieces
        .iter()
        .map(|piece| {
            piece
                .node_size
                .unwrap_or_else(|| infer_node_size(&piece.triangles, piece.node.volume))
        })
        .collect();
    let collider_shapes = pieces
        .iter()
        .map(|piece| piece.collider_shape.clone())
        .collect();

    Ok(ScenarioDesc {
        nodes,
        bonds,
        node_sizes,
        collider_shapes,
    })
}

fn resolve_mode(mode: BondingMode) -> Result<(u32, f32), AuthoringError> {
    match mode {
        BondingMode::Exact => Ok((0, 0.0)),
        BondingMode::Average { max_separation } if max_separation > 0.0 => Ok((1, max_separation)),
        BondingMode::Average { max_separation } => {
            Err(AuthoringError::InvalidMaxSeparation(max_separation))
        }
    }
}

fn infer_node_size(triangles: &[Vec3], volume: f32) -> Vec3 {
    if let Some(bounds) = compute_bounds(triangles) {
        return bounds;
    }

    let edge = volume.max(0.0).cbrt();
    Vec3::new(edge, edge, edge)
}

fn compute_bounds(triangles: &[Vec3]) -> Option<Vec3> {
    let first = triangles.first()?;
    let mut min = *first;
    let mut max = *first;

    for vertex in &triangles[1..] {
        min.x = min.x.min(vertex.x);
        min.y = min.y.min(vertex.y);
        min.z = min.z.min(vertex.z);
        max.x = max.x.max(vertex.x);
        max.y = max.y.max(vertex.y);
        max.z = max.z.max(vertex.z);
    }

    Some(Vec3::new(
        (max.x - min.x).max(1.0e-6),
        (max.y - min.y).max(1.0e-6),
        (max.z - min.z).max(1.0e-6),
    ))
}
