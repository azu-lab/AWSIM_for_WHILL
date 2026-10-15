"""
DreamerV3 の JAX 設定を、実行する GPU に合わせて調整する。

third_party/dreamerv3 (b65cf81) は jax 0.4.33 に固定されている。この版の XLA は
Blackwell 世代 (compute capability 12.x, RTX 50xx) で float16 の変換をコード生成できず、
"Compiling train and report..." の途中で

    Unsupported conversion from f16 to f16
    LLVM ERROR: Unsupported rounding mode for conversion.

と出て止まる (RTX 5090 で確認。XLA_FLAGS=--xla_gpu_enable_triton_gemm=false でも同じ)。
bfloat16 でも "Unsupported conversion from bf16 to f16" で同様に止まり、float32 なら通る。
そのため、GPU が compute capability 12 以上のときだけ compute_dtype を float32 に切り替える。
パラメータ自体は float32 で保持されるので、float16 で学習したチェックポイントも読める。
(DreamerV3 本家の main.py も compute_dtype に bfloat16 / float32 しか想定していない。)
"""

from typing import Optional


def _max_compute_capability() -> Optional[float]:
    import jax
    best = None
    for device in jax.devices():
        cc = getattr(device, "compute_capability", None)
        if cc:
            value = float(cc)
            best = value if best is None else max(best, value)
    return best


def adjust_jax_config(jax_config, compute_dtype: Optional[str] = None):
    """
    Args:
        jax_config: elements.Config (DreamerV3 の config.jax)
        compute_dtype: 明示指定 (float16 / bfloat16 / float32)。None なら自動。
    Returns:
        compute_dtype を確定させた jax_config
    """
    if compute_dtype is None:
        compute_dtype = jax_config.compute_dtype
        if compute_dtype in ("float16", "bfloat16"):
            cc = _max_compute_capability()
            if cc is not None and cc >= 12.0:
                print(f"[jax_compat] GPU compute capability {cc}: {compute_dtype} does not compile with "
                      f"jax 0.4.33 on this GPU, using float32 (override with --compute-dtype)")
                compute_dtype = "float32"
    return jax_config.update({"compute_dtype": compute_dtype})
