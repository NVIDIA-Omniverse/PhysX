// @ts-check
import { test, expect } from '@playwright/test';

const DEFAULT_BASE_URL = 'http://127.0.0.1:8000';
const BRIDGE_PATH = '/blast/js_stress_example/bridge-ext.html';

function makeConsoleListeners(page) {
  const logs = { errors: [], warns: [], info: [] };
  page.on('console', (message) => {
    const text = message.text();
    if (message.type() === 'error') {
      logs.errors.push(text);
      // Pipe to stderr for CI visibility
      // eslint-disable-next-line no-console
      console.error(`[browser][error] ${text}`);
    } else if (message.type() === 'warning') {
      logs.warns.push(text);
      // eslint-disable-next-line no-console
      console.log(`[browser][warn] ${text}`);
    } else if (message.type() === 'log') {
      logs.info.push(text);
      // eslint-disable-next-line no-console
      console.log(`[browser][log] ${text}`);
    }
  });
  return logs;
}

test.describe('bridge stress demo', () => {
  test('page reaches ready state without errors', async ({ page }, testInfo) => {
    const baseUrl = process.env.BRIDGE_BASE_URL ?? DEFAULT_BASE_URL;
    const url = new URL(BRIDGE_PATH, baseUrl).toString();

    const consoleLogs = makeConsoleListeners(page);

    page.on('pageerror', (err) => {
      const msg = `PageError: ${err?.message ?? err}`;
      consoleLogs.errors.push(msg);
      // eslint-disable-next-line no-console
      console.error(`[pageerror] ${msg}`);
    });
    page.on('requestfailed', (req) => {
      const failure = req.failure();
      // eslint-disable-next-line no-console
      console.error(`[requestfailed] ${req.url()} ${failure?.errorText ?? ''}`);
    });

    await page.goto(url, { waitUntil: 'domcontentloaded' });

    try {
      await page.waitForFunction(
        () => {
          const harness = globalThis.__bridgeExt;
          return harness && harness.ready === true && (harness.tickCount ?? 0) > 5;
        },
        { timeout: 120_000 }
      );
    } catch (e) {
      const snapshot = await page.evaluate(() => ({
        title: document.title,
        href: location.href,
        harness: globalThis.__bridgeExt ?? null
      }));
      // eslint-disable-next-line no-console
      console.error(`[diagnostic] waitForFunction timeout, snapshot: ${JSON.stringify(snapshot, null, 2)}`);
      throw e;
    }

    const harness = await page.evaluate(() => {
      const data = globalThis.__bridgeExt ?? {};
      return {
        ready: !!data.ready,
        tickCount: data.tickCount ?? 0,
        lastError: data.lastError ?? null,
        url: data.url ?? '',
        overstressed: data.overstressed ?? null,
        actorCount: data.actorCount ?? null,
        console: data.console ?? []
      };
    });

    if (harness.lastError) {
      consoleLogs.errors.push(`HarnessError: ${harness.lastError}`);
    }

    if (testInfo.attachments) {
      await testInfo.attach('bridge-harness.json', {
        body: JSON.stringify({ harness, consoleLogs }, null, 2),
        contentType: 'application/json'
      });
    }

    expect(consoleLogs.errors, consoleLogs.errors.join('\n')).toHaveLength(0);
    expect(harness.ready).toBeTruthy();
    expect(harness.tickCount).toBeGreaterThan(5);
  });
});
