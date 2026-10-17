import { readFile, writeFile } from 'node:fs/promises';
import { compile } from 'json-schema-to-typescript';
// json-schema-to-typescript uses draft-07 tuple syntax; contracts remain 2020-12.
function typeSchema(value) {
  if (Array.isArray(value)) return value.map(typeSchema);
  if (value && typeof value === 'object') {
    return Object.fromEntries(Object.entries(value).map(([key, item]) =>
      [key === 'prefixItems' ? 'items' : key, typeSchema(item)]));
  }
  return value;
}
for (const name of ['twin-state', 'connection-status', 'warehouse-scene']) {
  const schema = JSON.parse(await readFile(new URL(`../../../shared/schemas/${name}.schema.json`, import.meta.url)));
  const result = await compile(typeSchema(schema), schema.title, {
    bannerComment: '/* Generated from shared/schemas. Run npm run generate:types. */',
    additionalProperties: false,
  });
  const output = new URL(`../src/core/types/${name}.ts`, import.meta.url);
  if (process.argv.includes('--check')) {
    if (await readFile(output, 'utf8') !== result) throw new Error(`Type drift: ${name}`);
  } else await writeFile(output, result);
}
