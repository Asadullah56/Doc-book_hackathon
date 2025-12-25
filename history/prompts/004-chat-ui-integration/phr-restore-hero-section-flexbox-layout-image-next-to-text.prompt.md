---
id: PHR-0058
title: Restore Hero Section Flexbox Layout Image Next to Text
stage: refactor
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: 004-chat-ui-integration
branch: 004-chat-ui-integration
user: Asadullah56
command: /sp.implement
labels: [layout, hero, flexbox, css, docusaurus, refactor]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
- doc-book/src/pages/index.tsx
- doc-book/src/pages/index.module.css
tests:
- none
---

## Prompt

TASK: Restore Hero Section to original Flexbox layout (Image next to Text).

1. **Reset CSS**:
   - In `src/css/custom.css`, REMOVE `position: absolute` from `.heroImage`.
   - Set `.heroImage` back to:
     `display: flex; justify-content: center; align-items: center; width: auto; position: relative;`

2. **Restore JSX Layout**:
   - In `src/pages/index.tsx`, find the Hero `<header>`.
   - Ensure the container uses: `display: flex; flex-direction: row; align-items: center; justify-content: space-between;`.
   - Move the Image div BACK inside the main flex container so it sits to the right of the text, NOT floating in the corner.

3. **Image Scaling**:
   - Set the image class `pulseImage` back to: `max-width: 100%; height: auto;` without any absolute overrides.

4. **Cleanup**:
   - Ensure the Meta-Tag leak (title=...) is still removed from the top for a professional look.

## Response snapshot

Successfully restored the Hero Section to its original Flexbox layout with the image positioned next to the text. Removed absolute positioning from heroImage, reverted to flex layout with display: flex; justify-content: center; align-items: center; width: auto; position: relative. Restored JSX layout to use flex flex-col md:flex-row layout with the image in the same flex container as the text content. Image scaling is now max-width: 100% height: auto. Confirmed meta-tag leak was already cleaned up.

## Outcome

- ✅ Impact: Hero section restored to original flexbox layout with image next to text
- 🧪 Tests: none
- 📁 Files: doc-book/src/pages/index.tsx, doc-book/src/pages/index.module.css
- 🔁 Next prompts: none
- 🧠 Reflection: Returning to flexbox layout provides better semantic structure and responsive behavior

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): None needed - all requirements met