> [!IMPORTANT]
> **User Preferences (always respect these):**
> - The user writes or copy-pastes ALL code themselves. Do **NOT** automatically write, edit, or run code files.
> - Act as a thinking partner and explainer. Generate code in the chatbox for copy-pasting only.
> - Building markdown artifacts is fine. Modifying `.cpp`, `.h`, `.py`, `.m`, or any source files directly is **not**.
> - Do not run terminal commands that execute or modify code without explicit user permission.
> - **Coding workflow**: Claude (Sonnet/Opus) discusses the design and writes high-level logic/annotated pseudocode/algorithm in the chat. The user then prompts **Sonnet** to implement the actual code from Claude's spec. Sonnet/Opus = architect + reviewer; Sonnet = implementer.
> You only have permission to do changes in knowledge_base.md / some markdown file in the home folder which is our knowledge_base as we keep on working on project. 
> When giving code changes, give code changes in format (- for lines to delete, + for lines to add) ONLY. Dont give lines which are not changed, even in diff it has been noted entire sections with changes inside in some lines come up. only show lines with changes. 
> When giving code changes in - / + diff format, use ```diff fenced code blocks so deletions render red and additions render green.

# Additional coding guidelines
## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When (planning to) editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your code changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.
