# Writing Guide

This page explains how to write a chapter of the book. It is for anyone who wants to add or improve a chapter, including first-time contributors. The [Stanley section](path_tracking/stanley.md) is written with this guide and can be used as an example.

## What a chapter should do

A reader should understand **why** the algorithm works, **how** it is computed, and **where** each equation lives in the code, and then be able to change the code and see what happens. Being easy to follow is more important than being complete. If a topic needs a long detour, link to a reference instead.

## Structure of a chapter

Every chapter has the same sections, in two layers ([documentation roadmap](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/issues/165)):

**Theory layer**: doesn't depend on how the code is organized, so it can be written now.

1. **Problem and intuition**: what problem the algorithm solves, and the idea in a few sentences and a figure, before any equation.
2. **Theory and derivation**: the equations, derived step by step with the symbols of [Notation and Units](NOTATION.md).
3. **Algorithm**: the computation as numbered steps.
4. **Limits and references**: when it fails or needs tuning, what the implementation simplifies compared with the paper, and the references.
5. **Check your understanding**: two or three short questions that the reader answers from the text alone, without running code, each with its answer folded under it. Ask *why* or *what happens if*, not for a definition to repeat, for example "Why does the second term of the Stanley controller use $\arctan$ instead of $k e / v$ directly?"

**Code layer**: depends on the code structure, which the [code roadmap](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/issues/131) changes. Write it for a stage once its interface is stable.

6. **Where the equations live in the code**: each equation next to the code that computes it.
7. **Comparison experiments**: change a parameter or swap the algorithm, and show what changes.
8. **Exercises**: small tasks the reader can run.

**Pages.** A chapter is a folder (`doc/<topic>/`) with the chapter page `index.md`. Put each section on its own page in the folder and link the pages from the chapter page; the linked pages are nested under the chapter in the sidebar. A short chapter can be written on the chapter page alone, with its sections as `##` headings, like [Chapter 2](simulator/index.md).

**Being found by search.** Many readers arrive from a search engine, typing the algorithm's name with words such as "explained", "equations", "python", "example" or "github", or comparing two algorithms ("stanley vs pure pursuit", "ekf vs ukf", "mppi vs mpc"). The first sentence of each page becomes its description in search results and link previews (about 160 characters), so:
- Start each page with one sentence that names the algorithm (with its common abbreviation) and says what the page gives, for example "The Stanley controller for path tracking explained, with its equations and a Python simulation."
- Name comparison sections "A vs B" (for example "MPC vs MPPI"), the words people search for.

## Template

Copy this into a new page and fill in each section. Delete the comments.

````markdown
# N.M Algorithm Name

<!-- First a search sentence: the algorithm's name, abbreviation, and "explained, with its equations and a Python simulation".
     Then one or two sentences: what this section is about and what the reader will be able to do -->

## Problem and intuition

<!-- The problem, the idea, and a figure or GIF of the algorithm working -->

**Run it** from the repository root: `python src/simulations/<category>/<name>/<name>.py` ([code on GitHub](/src/simulations/<category>/<name>))

## Theory

<!-- Define every symbol at its first use, following NOTATION.md. Number the important equations -->

## Algorithm

```text
1. ...
2. ...
```

## Limits

<!-- When it fails, what needs tuning, and what this implementation simplifies -->

## Where the equations live in the code

<!-- For each equation: one sentence, then a literalinclude of the function that computes it -->

## Experiments

<!-- Which parameter to change, in which simulation, and what to look for -->

## Check your understanding

1. <!-- A "why" or "what happens if" question answered by the text above -->

   <details><summary>Answer</summary>

   <!-- Two or three sentences, pointing to the equation that explains it -->

   </details>

## Exercises

<!-- Two or three tasks, from easy to harder -->

## References

1. Author, "Title", Venue, Year. [PDF](URL) or [DOI](URL)
````

## Math and notation

- Use the symbols of [Notation and Units](NOTATION.md). If a reference uses different symbols, say so once and use the book's symbols.
- Define each symbol the first time it appears, with its unit.
- Write inline math with `$...$` and display math with `$$...$$`.
- Number an equation with `\tag{n}` at its end. If a numbered equation has a line break `\\` anywhere (several lines, or a matrix), write it in `\begin{equation*} ... \tag{n} \end{equation*}` without `$$`, with `aligned` inside for several lines: the book wraps `$$` math in a `split` environment, where `\tag` and `\\` don't work together.
- When the code and the paper differ (a sign, a simplification, a fixed parameter), say so in the text.

## Code in the text

Never copy code into the text. Copied code goes out of date when the code changes. Include it from the source with `literalinclude`, by class or function name with `:pyobject:`:

````markdown
```{literalinclude} ../../src/components/control/stanley/stanley_controller.py
:language: python
:pyobject: StanleyController._calculate_control_input
```
````

- The path is relative to the page.
- Include only the function the text explains, not the whole class.
- The directive shows as a code block on GitHub. Read and check the page in the web book.

## Figures

Explain the theory with figures as well as equations. Pick the kind of figure by what it shows:

| What it shows | Example | How to make it |
|---|---|---|
| Geometry | the bicycle model, the errors of a path tracking controller | a figure script |
| Concepts | a Gaussian, a covariance ellipse, the sigma points of the UKF, the shape of a cost function | a figure script |
| A process or a structure | the predict and update loop of a Kalman filter, the blocks of the system | a Mermaid diagram |
| The algorithm working | the vehicle following a course | the simulation's GIF |

**Figure scripts.** Put a Python script that draws the figure with matplotlib in a `figures/` folder next to the page. When the book is built, every `figures/*.py` is run and its figure is saved as an SVG next to it. The SVG files are not committed (`.gitignore`), so the figures always match the scripts and the repository doesn't grow. Include the figure with the `figure` directive; the text after the options is the caption:

````markdown
```{figure} figures/stanley_errors.svg
:alt: What the figure shows, for readers who can't see it

The caption. It can use math such as $\theta_e$.
```
````

- See [`path_tracking/figures/stanley_errors.py`](/doc/path_tracking/figures/stanley_errors.py) for an example.
- Use the colors of [`doc/figure_style.py`](/doc/figure_style.py), which have a fixed meaning in every figure (course, vehicle, errors, control inputs, estimates, measurements, obstacles). The font sizes and line widths are applied automatically.
- Label the figure with the same symbols as the equations, written as math (`r"$\theta_e$"`).
- A figure script can compute its geometry with the equations of the chapter, so the figure is right by construction. Fix the random seed if it uses random numbers.
- The figure shows as a plain directive on GitHub. Check it in the web book.

**Other rules**

- Don't copy figures from papers or other websites. Draw your own figure of the same idea, and cite the source.
- Use a GIF to show the algorithm working, and a figure script for a still image of a specific moment.
- Give every figure an alt text that says what it shows.

## Derivations and docstrings

- Long derivations go in the book.
- Docstrings keep the key equations next to the code, with a link to the book section.

## References

- List them at the end of the chapter, numbered, in the form: Author, "Title", Venue, Year, followed by a link such as `[PDF](URL)` or `[DOI](URL)`. Don't write a bare URL: a long URL doesn't wrap and runs off the page.
- Cite them in the text by number, for example [1].
- Prefer the original paper, and add a freely available version when there is one.

## Checking a chapter

Build the book locally as described in [HOWTOCONTRIBUTE.md](https://github.com/ShisatoYano/AutonomousVehicleControlBeginnersGuide/blob/main/HOWTOCONTRIBUTE.md#add-documentation-about-existing-program), and check that:

- the build has no warnings,
- every equation renders and every included function is the one the text talks about,
- the figures match the current code.
