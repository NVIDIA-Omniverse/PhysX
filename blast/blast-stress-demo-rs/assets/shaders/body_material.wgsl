#import bevy_pbr::forward_io::VertexOutput
#import bevy_pbr::mesh_view_bindings::view

struct BodyMaterial {
    color: vec4<f32>,
};

@group(#{MATERIAL_BIND_GROUP}) @binding(0) var<uniform> material: BodyMaterial;

@fragment
fn fragment(in: VertexOutput) -> @location(0) vec4<f32> {
    let n = normalize(in.world_normal);
    let l = normalize(vec3<f32>(-0.45, 0.88, 0.30));
    let v = normalize(view.world_position - in.world_position.xyz);
    let h = normalize(l + v);

    let ambient = 0.18;
    let diffuse = max(dot(n, l), 0.0) * 0.70;
    let specular = pow(max(dot(n, h), 0.0), 32.0) * 0.25;
    let lighting = ambient + diffuse + specular;

    return vec4<f32>(material.color.rgb * lighting, material.color.a);
}
