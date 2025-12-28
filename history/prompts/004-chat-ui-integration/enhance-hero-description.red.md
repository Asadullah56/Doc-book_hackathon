---
id: 20251228234049
title: enhance-hero-description
stage: red
date: 2025-12-28T23:40:49+00:00
surface: CLI
model: claude-sonnet-4-5-20250929
feature: 004-chat-ui-integration
branch: 004-chat-ui-integration
user: asada
command: /sp.implement
labels: [hero, typography, animation, css, react]
links:
  spec: ../specs/004-chat-ui-integration/spec.md
  ticket:
  adr:
  pr:
files:
  - path: doc-book/src/pages/index.tsx
    role: main
    impact: major
  - path: doc-book/src/pages/index.module.css
    role: main
    impact: major
tests:
  - type: manual
    path: doc-book/src/pages/index.tsx
    summary: Browser verification of hero section
---

## Prompt

TASK: Enhance Hero Description with detailed text, Typewriter animation, and Glassmorphism styling.

1. **Detailed Description & Typewriter**:
   - In `src/pages/index.tsx`, update the description text to:
     "The ultimate technical manual for building the future. Master fundamental ROS 2 foundations, integrate advanced AI brains, and orchestrate complex humanoid systems through a structured 4-module journey."
   - Add a 'typewriter' effect using CSS keyframes or a simple React span class `animate-pulse` to the last word to make it feel "active."

2. **Typography Styling**:
   - Wrap the description in a div with these Tailwind classes:
     `text-gray-200 text-lg md:text-2xl max-w-3xl mx-auto leading-relaxed drop-shadow-md mb-8`.
   - Highlight keywords like "ROS 2", "AI brains", and "humanoid systems" in `text-bot-cyan`.

3. **Background Image Integration**:
   - Set `static/img/hero-background.jpg` as the full-page background with a 70% dark overlay to ensure text is readable.
   - Use `background-attachment: fixed` to give a subtle "Parallax" effect when scrolling.

4. **Animated Button**:
   - Update the "Explore the Library" button to have a pulsing glow:
     `bg-transparent border-2 border-bot-cyan text-bot-cyan hover:shadow-[0_0_30px_#00f2ff] animate-bounce-subtle`.

5. **Structure**:
   - Ensure the layout is: [Title] -> [Animated Subtitle] -> [Detailed Description] -> [Explore Button].

## Response snapshot

Successfully implemented all Hero Description enhancements:

1. Updated `index.tsx` with detailed description text including highlighted keywords ("ROS 2", "AI brains", "humanoid systems") and a blinking cursor on the word "journey"
2. Added comprehensive CSS styling in `index.module.css` including:
   - Background image with parallax effect using `background-attachment: fixed`
   - 70% dark overlay for text readability
   - Typography styling with responsive font sizes
   - Typewriter cursor animation
   - Highlighted keywords in bot-cyan color
   - Transparent button with cyan border and pulsing glow effect
   - Subtle bounce animation for the button
3. Used existing `module-bg.jpg` as hero background since `hero-background.jpg` didn't exist
4. Verified correct structure: Title -> Animated Subtitle -> Detailed Description -> Explore Button

## Outcome

- ✅ Impact: Major enhancement to homepage hero section with improved visual appeal and engagement
- 🧪 Tests: Manual verification required - open browser to confirm styling and animations
- 📁 Files: 2 files modified (index.tsx, index.module.css)
- 🔁 Next prompts: None specified
- 🧠 Reflection: All requirements implemented successfully. User can replace hero-background.jpg if desired for a different background image.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add actual typewriter animation that types out the full text instead of just a blinking cursor
