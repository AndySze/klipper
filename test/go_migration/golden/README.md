# Generated Golden Outputs

This directory is populated by:

```sh
./scripts/go_migration_golden.py gen --dictdir dict
```

It is intentionally ignored by git (except this README), because
goldens depend on the dictionary bundle version and local environment.

If/when you want to commit goldens for CI, we can switch the ignore
rules and pin the dictionary bundle version explicitly.
