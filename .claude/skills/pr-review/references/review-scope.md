# Review Scope

Which parts of a PR the automated review covers, and which the maintainer checks by eye. The automated review always does the local verification, the convention checks and the comment draft. The maintainer checks what decides the project's direction and content, so they keep an accurate picture of the project.

Classify a PR by its changes. More than one type can apply; then the maintainer checks the union.

| Type | Changes | The maintainer checks by eye |
|---|---|---|
| A. Minor | Docstrings, typos, comments, tests only, CI | Only the review summary |
| B. New algorithm | A new simulation or algorithm | The demo GIF, the core algorithm function(s), and whether it is easy to understand as learning material |
| C. Shared components | Changes to existing code under `src/components/` | The whole diff, since every simulation using the component changes, and the last frames on `main` and the PR side by side |
| D. Book text | Text under `doc/` | The text itself, since it becomes the book |
| E. Design or scope | Accepting or declining a proposal, a new interface, the architecture or the roadmap | The decision. The review only gives the options and the evidence |
