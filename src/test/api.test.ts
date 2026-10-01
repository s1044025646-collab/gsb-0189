import { test } from "node:test";
import assert from "node:assert/strict";
import { mkdtempSync } from "node:fs";
import { tmpdir } from "node:os";
import { join } from "node:path";
import type { AddressInfo } from "node:net";
import { CalibService } from "../service.js";
import { createApp } from "../server.js";

async function withServer(fn: (base: string) => Promise<void>): Promise<void> {
  const dir = mkdtempSync(join(tmpdir(), "pinhole-api-"));
  const service = new CalibService({ dataDir: dir });
  const server = createApp(service);
  await new Promise<void>((res) => server.listen(0, res));
  const base = `http://localhost:${(server.address() as AddressInfo).port}`;
  try {
    await fn(base);
  } finally {
    await new Promise<void>((res) => server.close(() => res()));
    service.store.close();
  }
}

async function post(base: string, path: string, body: unknown): Promise<any> {
  const res = await fetch(base + path, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify(body),
  });
  const text = await res.text();
  return JSON.parse(text);
}

test("API 健康检查与直接标定往返", async () => {
  await withServer(async (base) => {
    const health = await (await fetch(base + "/api/health")).json();
    assert.equal(health.ok, true);

    const gen = await post(base, "/api/synthetic/generate", {
      config: { seed: 11, pointCount: 50, noiseSigmaPx: 0, intrinsics: { fx: 700, fy: 700, cx: 300, cy: 200, k1: 0, k2: 0 } },
    });
    const cal = await post(base, "/api/calibrate/direct", {
      observations: gen.observations,
    });
    assert.equal(cal.result.diagnostics.identifiable, true);
    assert.ok(Math.abs(cal.result.intrinsics.fx - 700) < 1e-4);

    const bp = await post(base, "/api/backproject", {
      intrinsics: cal.result.intrinsics,
      pixels: [{ u: 300, v: 200 }],
      depths: [2],
    });
    assert.equal(bp.results[0].converged, true);
    assert.ok(Math.abs(bp.results[0].point.z - 2) < 1e-12);
  });
});
