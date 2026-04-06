export const GRAVITY_DEFAULT = -9.81;
export const PROJECTILE_SPEED = 35;
export const MAX_EVENTS = 8;
export const STRENGTH_DEFAULT = 0.05;
export const SOLVER_DEFAULTS = Object.freeze({
    maxIterations: 32,
    toleranceExponent: -6
});
export const LIMITS_DEFAULTS = Object.freeze({
    compressionElasticLimit: 3.5e5,
    compressionFatalLimit: 6.5e5,
    tensionElasticLimit: 2.8e5,
    tensionFatalLimit: 4.5e5,
    shearElasticLimit: 1.8e5,
    shearFatalLimit: 3.5e5
});
export const BRIDGE_DIMENSIONS = Object.freeze({
    deckSegmentLength: 9,
    deckWidth: 4,
    deckThickness: 0.6,
    deckElevation: 2.2,
    pierWidth: 1.4,
    pierDepth: 2.6,
    pierHeight: 2.2
});
export const CAR_PROPS = Object.freeze({
    mass: 2400,
    length: 3.6,
    width: 1.8,
    height: 1.2,
    speed: 4.5
});
