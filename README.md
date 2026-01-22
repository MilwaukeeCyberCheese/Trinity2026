# Trinity

The Milwaukee Cyber Cheese's 2026 robot code, Trinity.

## Code Best Practices

- Use [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/). TL;DR: `verb: Short description`, `verb(scope): Short description`, or in the case of large/breaking changes, `verb(scope)!: Short description`.
- Name branches according to Conventional Commits verbs - for instance in a PR handling documentation, name the branch `docs/change-in-a-few-words`.
  - Don't name branches `YourUsername-patch-1`!
- Each PR should be one change or one group of like changes. For instance, changing the documentation AND changing drive settings should be two seperate PRs and not one with both changes.
- In the case of significant merge conflicts, do not attempt to resolve them yourself and instead get a codeowner to do it for you.
- List of verbs that are acceptable to use: `feat`, `docs`, `chore`, `refactor`, `fix`, `wip`
- List of scopes that are acceptable to use: `auto`, `teleop`, `build`. You can use more for your specific scope, but verify then with a codeowner first.
- Subsystems should have hardware abstracted so they can be simulated both with the WPIlib simulator and with AdvantageKit
- Commands should handle exactly one task and do it well, without blocking and without causing loop overruns (50hz).
- Code should be linted using Spotless. To lint the code locally, run `./gradlew spotlessApply`.
