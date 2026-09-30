---
name: pr-review
description: Use when reviewing a pull request to this repository on the maintainer's behalf, or re-reviewing it after the contributor pushed fixes. Covers local verification (tests, runtime, demo GIF), project-specific review points, and an English comment draft. Contributors can also use it to self-review before opening a PR. Delegated from the maintainer's `pr-workflow` for this repository; never posts comments or changes PR state.
---

# PR Review

Reviews a pull request against this project's conventions and its purpose as learning material, verifies it locally, and prepares findings and an English comment draft. Posting, approving and merging are always done by the maintainer.

## Prerequisites

- Read `CLAUDE.md` and `HOWTOCONTRIBUTE.md` first. They define the conventions checked here
- Never run commands that change PR state (`gh pr review`, `gh pr comment`, `gh pr merge`). Draft text only
- Don't switch branches or create worktrees in the user's checkout. Export the PR snapshot with `git archive <sha> | tar -x -C <temp dir>` into a temporary directory (the session scratchpad if available) and run everything there
- Base every finding on something checked (code read, test run, measurement). Mark anything unverified as a question, not a finding

## Understanding the PR

1. Get the PR body, commits and changed files (`gh pr view <number> --json body,commits,files,headRefOid`), and read the linked issue to see what was agreed with the maintainer
2. If the PR is stacked on other open PRs, review only the commits of this step (`git show <sha>`), since earlier steps are reviewed in their own PRs
3. If the body has images or videos (`https://github.com/user-attachments/assets/...` or `![...](...)`), tell the user they need to check them in the browser

## Local verification

1. Fetch the head (`git fetch origin pull/<number>/head`) and export it with `git archive` as described above
2. Run the tests added or changed by the PR with `pytest --durations=0`
3. Compare the new simulation's runtime with the closest existing one (same scenario or category). If it is clearly slower, profile it with `cProfile` to find the cause, and try the fix in the temporary copy so the finding can include measured numbers
4. Copy the current `generate_example_gallery.py` from main into the export and run `generate()`, because old PR branches may not have the latest docstring validation
5. Extract a frame from each demo GIF (Pillow `Image.seek`) and look at it to check that it shows the algorithm working
6. Check conflicts with main with `git merge-tree --write-tree origin/main <sha>`, and whether CI has run or still needs the maintainer's approval (fork PRs show `action_required`)

## Review

1. Check the changes against `references/review-checklist.md`
2. When reviewing the algorithm, read the connected existing components (e.g. how the LiDAR computes angles) instead of assuming their behavior
3. Rank findings: requests needed before merge first, then suggestions, then minor points. For each, give the cause, a concrete fix and the evidence (numbers, file:line)
4. Also list what is good, so the draft can open with it

## Drafting the comment

1. Write the draft in English, starting with thanks and good points, then numbered requests and suggestions
2. Keep each item short: the problem, the measured impact, and the suggested fix
3. Show the Japanese summary of findings and the English draft to the user, and remind them that posting and approving are theirs to do

## Re-review

1. List the commits added since the last review and check that each earlier finding is addressed, fixed or reasonably declined (a documented trade-off counts if its reasoning is correct; verify it)
2. Rerun the local verification for the changed parts and report numbers against the previous ones
3. Conclude whether the PR can be merged, and draft a short follow-up comment
4. After the maintainer merges, check that `Update_Examples_Gallery` ran and that the bot's commit added the entry as expected
