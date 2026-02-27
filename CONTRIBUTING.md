# Contributing to Arduino Robot Arm

This guide covers the git workflow, commit conventions, and code review process for contributors.

## Branching Strategy

We use a **two-tier branch workflow**: feature branches integrate into `dev`; only tested, stable code is promoted to `main`.

```
main ──────────────────────────────────────────────────── (stable, tagged releases)
  └─ dev ─────────────────────────────────────── MR ──┘  (integration branch)
       └─ feature/serial-parser ──── MR ──┘
       └─ fix/shoulder-limit-clamp ── MR ──┘
       └─ docs/wiring-diagram ─────── MR ──┘
```

| Branch | Purpose |
|--------|---------|
| `main` | Production-ready firmware. Protected — no direct pushes. Every commit here is a release candidate. |
| `dev`  | Shared integration branch. All feature/fix/docs MRs target `dev`. Merged into `main` once stable. |
| `feature/*`, `fix/*`, … | Short-lived work branches, always cut from `dev`. |

### Branch Naming

| Prefix     | When to use                              | Example                        |
|------------|------------------------------------------|--------------------------------|
| `feature/` | New functionality or skill               | `feature/kinematics-solver`    |
| `fix/`     | Bug or hardware behaviour fix            | `fix/gripper-pwm-glitch`       |
| `docs/`    | Documentation only                       | `docs/calibration-table`       |
| `test/`    | New or updated tests                     | `test/native-servo-limits`     |
| `refactor/`| Code restructuring without behaviour change | `refactor/extract-joint-map` |
| `chore/`   | Build config, CI, tooling changes        | `chore/update-platformio-ini`  |

Rules:
- Always cut new branches from `dev`, not `main`.
- Keep branches short-lived — merge or close within a sprint.
- One logical concern per branch.
- Only merge `dev` → `main` after all CI checks pass and the build is hardware-validated.

## Commit Messages

Use [Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<scope>): <short summary>

[optional body — explain WHY, not WHAT]

[optional footer: Refs #123, BREAKING CHANGE: ...]
```

**Types:** `feat`, `fix`, `docs`, `test`, `refactor`, `chore`, `perf`

**Scope:** component or subsystem (e.g. `gripper`, `serial`, `kinematics`, `limits`, `ci`)

### Examples

```
feat(serial): add HOME command to reset all joints

fix(limits): clamp shoulder angle to [30, 150] to protect gears

docs(wiring): add power supply diagram for 6-servo setup

chore(ci): add PlatformIO native build step to GitLab CI

BREAKING CHANGE: setAngle() now takes joint enum instead of pin number
```

Rules:
- Keep the subject line under **72 characters**.
- Use the **imperative mood** ("add", "fix", "update" — not "added" or "fixes").
- Mark breaking changes with `BREAKING CHANGE:` in the footer.
- Reference issues/tickets with `Refs #<id>` or `Closes #<id>`.

## Merge Request (MR) Process

1. **Push your branch** and open an MR against `main`.
2. Fill in the MR description: what changed, why, and how to test it.
3. Ensure the pipeline passes (build + native tests).
4. Request review from at least **one other developer**.
5. Address all review comments before merging.
6. Use **Squash and Merge** for small features; **Merge Commit** for larger ones where individual commits are meaningful.
7. Delete the source branch after merge.

### MR Checklist

- [ ] `pio run` succeeds (no build errors or warnings added)
- [ ] `pio test -e native` passes (if applicable)
- [ ] Angle limits defined as `#define` constants in `main.cpp`
- [ ] No `delay()` calls in `loop()` — use `millis()` timers
- [ ] Serial baud rate is 115200
- [ ] New pins or joints documented in `AGENTS.md`
- [ ] Commit messages follow Conventional Commits

## Code Review Guidelines

**For authors:**
- Keep MRs small and focused — reviewers should be able to understand the change in under 15 minutes.
- Add context in the MR description for non-obvious decisions.
- Respond to all comments (resolve or explain why not changed).

**For reviewers:**
- Review within **2 working days**.
- Distinguish blocking issues (must fix) from suggestions (nice to have) — prefix suggestions with `nit:`.
- Approve once all blockers are resolved; do not re-request changes for nits.

## Local Development Setup

```bash
# Clone
git clone <repo-url>
cd arduino_robot_arm

# Build firmware
pio run

# Run native unit tests (no hardware needed)
pio test -e native

# Flash to connected UNO
pio run --target upload

# Monitor serial output
pio device monitor
```

## Adding a New Skill (Library)

```
lib/
└── <skill_name>/
    ├── library.json        ← PlatformIO library manifest
    └── src/
        ├── <skill_name>.h
        └── <skill_name>.cpp
```

1. Create the folder structure above under `lib/`.
2. Include in `src/main.cpp`: `#include <<skill_name>.h>`.
3. Document the skill's API and pin usage in `AGENTS.md`.
4. Add native tests under `test/` where logic is hardware-independent.

## Questions

Open an issue or start a discussion in the repository.
