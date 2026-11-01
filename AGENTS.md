# Canonical repository

- User instruction dated 2026-10-18: only push updates to the user-designated canonical repository. Other repositories are historical/read-only for this workflow.
- Reusable packages are maintained directly under `vendor/verilog-reusable-modules/` here. Do not publish changes back to the old source repository.
- Preserve unrelated local changes and other contributors' work. Application J6 slot mapping is defined by `rtl/rc/ht10a_input.v` and `doc/HT10A接线与验收.md`; standalone module examples use different mappings and must not overwrite it.
- Follow `vendor/ORIGIN.md` for application regression after vendor updates. Distinguish module/standalone-example verification from whole-vehicle hardware validation.
- One task, one commit (user instruction dated 2026-10-18): keep the code change and the documentation updates it requires in a single `git commit`. Do not split a task into a feature commit followed by a docs or hash-sync commit. Before committing, write any self-referential documentation field (current HEAD hash, ahead/behind counts, the title of the commit being created) so that it does not depend on the commit not yet made; otherwise it is stale the moment the commit lands and forces a second commit. If something was missed and the commit is not yet pushed, use `git commit --amend`; never amend a commit that is already pushed.

- User instruction dated 2026-10-21: before every commit, pull the latest changes from the shared repository. Preserve local work, resolve any conflicts, and rerun affected checks before committing. Do not push unless requested.
