/** @type {import('@playwright/test').PlaywrightTestConfig} */
const config = {
  testDir: '.',
  timeout: 120_000,
  use: {
    baseURL: process.env.BRIDGE_BASE_URL ?? 'http://127.0.0.1:8000',
    launchOptions: {
      args: [
        '--use-gl=swiftshader',
        '--ignore-gpu-blocklist',
        '--enable-webgl'
      ]
    }
  },
  projects: [
    {
      name: 'chromium',
      use: { browserName: 'chromium' }
    }
  ]
};

export default config;

