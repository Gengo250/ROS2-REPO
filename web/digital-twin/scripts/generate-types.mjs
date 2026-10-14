import { readFile, writeFile } from 'node:fs/promises';
import { compile } from 'json-schema-to-typescript';
for (const name of ['twin-state', 'connection-status']) {
  const schema = JSON.parse(await readFile(new URL(`../../../shared/schemas/${name}.schema.json`, import.meta.url)));
  const result = await compile(schema, schema.title, {
    bannerComment: '/* Generated from shared/schemas. Run npm run generate:types. */',
    additionalProperties: false,
  });
  const output = new URL(`../src/core/types/${name}.ts`, import.meta.url);
  if (process.argv.includes('--check')) {
    if (await readFile(output, 'utf8') !== result) throw new Error(`Type drift: ${name}`);
  } else await writeFile(output, result);
}
