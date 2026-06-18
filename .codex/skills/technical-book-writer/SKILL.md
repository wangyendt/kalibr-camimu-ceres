---
name: technical-book-writer
description: Write rigorous, readable, book-like technical documentation for mathematical, robotics, calibration, state-estimation, optimization, and code-derived derivations. Use when the user asks for thesis-style chapters, from-scratch explanations, story-telling technical books, derivation-heavy knowhow, symbol tables, chapter plans, formula/Jacobian explanations, or when existing notes feel like answer dumps, changelogs, code comments, or unexplained final results.
---

# Technical Book Writer

## Purpose

Use this skill to turn difficult technical material into a coherent book chapter, not a note dump. The output should read like a careful graduate textbook: concepts appear only after prerequisites, formulas are derived before they are used, and source-code facts are separated from theory.

## Core Workflow

1. Audit the reader path before writing.
   - List the concepts the chapter needs.
   - Order them by dependency: primitives, notation, model, residual, Jacobian, source mapping.
   - Move anything that uses an undefined concept later, or add a short working definition first.
   - Model the reader's background explicitly. If the reader knows the theory but not the implementation, treat implementation concepts as first-class teaching targets.

2. Establish the chapter spine and section contracts.
   - Write the chapter's main chain explicitly, such as data -> prediction -> residual -> tangent -> variable-block Jacobian.
   - At the start of each major section, state what object the section studies, what input it receives from previous sections, what output it produces for later sections, and where it sits on the chapter spine.
   - For derivation sections, state what is held fixed and what is perturbed before showing the first Jacobian.
   - For implementation sections, state whether the section is introducing a new model or only mapping an already-derived formula to source code.
   - Avoid isolated local explanations. Add one backward pointer and one forward pointer when a section could otherwise feel detached from the chapter goal.

3. Define a symbol ledger.
   - Define each symbol once, close to first use.
   - Keep one notation system as the book language.
   - Put code variable names in a bridge table instead of mixing them into derivations.
   - For competing conventions, state which convention is default and how to convert.

4. Explain unfamiliar implementation concepts before using them.
   - When introducing project-specific architecture, naming, update rules, graph abstractions, source conventions, or nonstandard design choices, give a short conceptual explanation before relying on them in formulas.
   - If the reader is familiar with the theory source, such as Micro Lie Theory, but unfamiliar with the implementation, bridge from the familiar theory to the implementation concept.
   - Use minimal examples when a source concept is not obvious from mathematics alone.
   - Separate "what the implementation stores or updates" from "what the mathematical derivation differentiates against."

5. Write as a story, not as a changelog.
   - Do not mention "previous version", "we changed", "this patch", or process history inside the book body.
   - Put revision notes in conversation summaries, plans, or commit messages, not in the chapter.
   - Begin each chapter with the reader's problem: what they do not yet understand and what the chapter will make clear.
   - Prefer declarative textbook prose over meta-commentary. Replace repeated phrases like "this section only answers", "in other words", "the following figure", or "as mentioned above" with direct transitions that state the mathematical object and the next step in the chain.
   - Keep reader-navigation sentences, but make them part of the argument: "projection maps camera points to predicted pixels" reads better than "this section discusses projection."

6. Derive before presenting final formulas.
   - Start from a simple physical or geometric statement.
   - Introduce the smallest useful formula.
   - Expand first order terms explicitly when signs matter.
   - Name the source of each minus sign: residual direction, frame direction, perturbation convention, or algebraic identity.

7. Separate theory from implementation.
   - Use theory notation for the main derivation.
   - Add a bridge table for source variables and functions.
   - Explain implementation-specific variants after the standard theory, unless the chapter is explicitly about code.
   - When source behavior differs from the theory convention, write the conversion formula.

8. Re-read as a dependency graph.
   - Scan from top to bottom and flag first uses of every symbol and concept.
   - Ensure no section depends on a later definition.
   - Check that examples and formulas use the current symbol ledger.
   - Remove orphaned details that do not answer the chapter's core question.

9. Reconnect context after every edit.
   - After modifying a section, scan the local subsection, the whole chapter, adjacent chapters, and cross-chapter references for related concepts that now need updates.
   - Update nearby definitions, summary bullets, source mapping tables, appendices, and chapter plans when the edit changes terminology or reader assumptions.
   - Add a short forward or backward pointer when two separated derivations depend on the same idea.
   - Avoid leaving isolated explanations that solve one local question but do not connect to the book's narrative.

## Book Polish Heuristics

- Keep convention dictionaries in foundation chapters and application-specific uses in application chapters. For example, derive a Micro-to-source bridge such as `boxMinus` or `boxTimes` once in the notation chapter, then briefly cite and specialize it in a camera or IMU residual chapter.
- At the first appearance of a named object, define the object before using its formula. For example, define target point, camera point, distortion mapping, parameter block, residual direction, and tangent convention before showing their Jacobians.
- Separate the chapter's forward chain from its Jacobian chain. First write the data flow, then the variation flow, then the residual sign, then the variable-block propagation.
- Use tables as ledgers, not substitutes for derivation. A table should summarize dimensions, source entries, or final Jacobians after the surrounding prose has explained why those rows exist.
- Figures should be introduced by their role in the derivation, not by generic wording. State which node, edge, or branch the figure clarifies.
- Source snippets belong after the reader can already understand the formula. State whether the snippet constructs the expression, computes the residual, or propagates a local Jacobian.
- When tightening prose, preserve useful signposts but remove repetitive scaffolding. Avoid starting every section with the same template; vary between physical intuition, chain position, perturbation setup, and code bridge depending on what the section needs.

## Chapter Shape

Use this order for derivation-heavy chapters:

1. Reader question: what confusion this chapter resolves.
2. Physical intuition: what is being compared or transformed.
3. Minimal symbols: frames, variables, dimensions, and conventions.
4. Standard theory: clean derivation independent of source code.
5. Convention bridge: local/global, left/right, active/passive, or other variants.
6. Code bridge: variable names, functions, expression nodes, or files.
7. Final formula: now justified by the derivation.
8. Common traps: only those that follow from the derivation.
9. Chapter summary: compact takeaways, no revision history.

## Formula Discipline

- Use bold uppercase for matrices and group elements, bold lowercase for vectors, and scalar lowercase for time and indices unless the project defines otherwise.
- State dimensions when a symbol first appears if ambiguity is likely.
- Before a section's first displayed formula, explain the formula's role in the chapter chain unless the role was stated immediately above.
- Do not show a final Jacobian without the perturbation definition it differentiates against.
- Do not mix residual minus signs with perturbation convention signs.
- For Lie groups, say whether tangent vectors are local/right or global/left.
- If using a code-specific perturbation, give it a distinct notation and define its relation to the theory notation.
- For Jacobians, write what is held fixed and what is perturbed.
- For implementation-specific concepts, first explain the concept in plain terms, then map it to source names.

## Review Checklist

Before finishing a chapter, verify:

- The opening reads like a book, not a work log.
- Each major section states its local object, input, output, and relationship to the chapter's main chain.
- Every concept appears after its prerequisites.
- Every symbol is defined before use.
- Formula signs are traced to explicit causes.
- Theory notation and source-code variables are separated.
- Source-code evidence appears after the reader can understand the theory.
- Project-specific concepts are introduced before their formulas depend on them.
- Tables and summaries reinforce the narrative rather than replacing derivation.
- The chapter can be read linearly without jumping forward for definitions.
- Related sections, adjacent chapters, appendices, symbol tables, and source bridges were checked after edits so the book remains connected rather than a set of isolated answers.
