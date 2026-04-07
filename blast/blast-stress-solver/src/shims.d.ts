declare module '*.cjs' {
  const factory: (options?: Record<string, unknown>) => any;
  export default factory;
}

declare module '*.mjs' {
  const factory: (options?: Record<string, unknown>) => any;
  export default factory;
}


