---
id: PHR-0057
title: Fix Hero Layout Robot Bottom Right Corner
stage: refactor
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: 004-chat-ui-integration
branch: 004-chat-ui-integration
user: Asadullah56
command: /sp.implement
labels: [layout, hero, positioning, css, docusaurus, refactor]
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

TASK: Fix Hero Layout - Move Robot to Bottom Right Corner.

1. **Hero Container**:
   - In `src/pages/index.tsx`, set the Hero `<header>` to `position: relative` and `overflow: hidden`.
   - Set padding to `py-20 md:py-32`.

2. **Robot Position**:
   - Wrap the Hero Image in a `div` with these classes:
     `absolute bottom-0 right-0 w-full max-w-[300px] md:max-w-[500px] opacity-80 z-0`.
   - Ensure the image has `object-fit: contain` and `pointer-events-none`.

3. **Text Layer**:
   - Wrap the Title, Subtitle, and Button in a `div` with `relative z-10` to keep them above the image.

4. **Meta Fix**:
   - Remove the stray text `title=...` from the top of the file.

## Response snapshot

Successfully restructured the Hero section layout to position the robot image in the bottom right corner. Updated the padding to py-20/md:py-32, implemented absolute positioning for the robot image at the bottom right, added proper z-index layering to keep text content above the image, and ensured the image has proper styling with object-fit contain and pointer-events-none. The layout now has the robot positioned as a background element while keeping the text content layered above it.

## Outcome

- ✅ Impact: Robot image now positioned in bottom right corner with proper layering
- 🧪 Tests: none
- 📁 Files: doc-book/src/pages/index.tsx, doc-book/src/pages/index.module.css
- 🔁 Next prompts: none
- 🧠 Reflection: Using absolute positioning with z-index layering creates effective visual hierarchy in the hero section

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): None needed - all requirements met