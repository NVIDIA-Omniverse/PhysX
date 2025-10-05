export function scaleStressLimits(limits, scale) {
  if (!limits) {
    throw new Error('scaleStressLimits requires limit values');
  }
  const resolvedScale = Number.isFinite(scale) ? scale : 1.0;
  return {
    compressionElasticLimit: limits.compressionElasticLimit * resolvedScale,
    compressionFatalLimit: limits.compressionFatalLimit * resolvedScale,
    tensionElasticLimit: limits.tensionElasticLimit * resolvedScale,
    tensionFatalLimit: limits.tensionFatalLimit * resolvedScale,
    shearElasticLimit: limits.shearElasticLimit * resolvedScale,
    shearFatalLimit: limits.shearFatalLimit * resolvedScale
  };
}

export function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

export function formatStrength(value) {
  return `${(value * 100).toFixed(1)}%`;
}

export function toleranceFromExponent(exponent) {
  return 10 ** exponent;
}

