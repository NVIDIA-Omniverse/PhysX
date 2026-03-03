# ovphysx

ovphysx is a self-contained Python library for USD-based physics simulation
with DLPack tensor interoperability. It wraps NVIDIA PhysX and provides:

- USD scene loading and rigid-body / articulation simulation
- Zero-copy tensor exchange with NumPy, PyTorch, and other DLPack frameworks
- Environment cloning for batched reinforcement-learning workloads

## Quick start

```bash
pip install ovphysx
```

```python
from ovphysx import PhysX

physx = PhysX()
physx.add_usd("scene.usda")
physx.step(1.0 / 60.0, 0.0)
physx.release()
```

For full documentation and tutorials, see `ovphysx/docs/markdown/` inside the
installed package, or the [ovphysx repository](https://github.com/NVIDIA-Omniverse/PhysX).
