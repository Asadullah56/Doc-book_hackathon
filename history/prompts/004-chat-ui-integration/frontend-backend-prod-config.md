---
id: 004-chat-ui-integration-green-001
title: frontend-backend-prod-config
stage: green
date: 2025-12-28
surface: cli
model: claude-sonnet-4-5
feature: 004-chat-ui-integration
branch: 004-chat-ui-integration
user: asada
command: /sp.implement
labels: [production, configuration, env-vars, cors, frontend-backend]
links:
  spec: ../specs/004-chat-ui-integration/spec.md
  ticket:
  adr:
  pr:
files:
  - path: doc-book/.env.local
    action: created
  - path: doc-book/docusaurus.config.ts
    action: modified
  - path: doc-book/src/components/ChatBubble.tsx
    action: modified
tests:
  - status: manual_verification_needed
    details: Verify frontend uses backendUrl from env and handles connection errors
---

## Prompt

Configure Frontend to Backend connection for production deployment.

1. **Environment Variables**:
   - Create or update `.env.local` for development and ensure there is a mechanism for production variables.
   - Add `SITE_BACKEND_URL=https://your-backend-api.com` (Replace with your actual backend URL).
   - In `docusaurus.config.ts`, use `customFields` to expose this URL:
     `customFields: { backendUrl: process.env.SITE_BACKEND_URL }`.

2. **API Client Setup**:
   - In `src/components/ChatBubble.tsx` (or wherever you call the API), replace hardcoded localhost URLs with:
     `const backendUrl = siteConfig.customFields.backendUrl;`
   - Ensure all `fetch()` or `axios` calls use this dynamic `backendUrl`.

3. **CORS & Headers**:
   - Check the backend configuration. Ensure the frontend production domain (e.g., https://your-humanoid-academy.vercel.app) is added to the Allowed Origins.
   - Ensure `Content-Type: application/json` is set in headers for POST requests.

4. **Error Handling**:
   - Add a "Loading" state and a "Backend Unavailable" error message in the UI so users know if the connection fails.

## Response snapshot

Successfully configured the frontend to backend connection for production deployment:

1. **Created `.env.local`** with `SITE_BACKEND_URL` variable configured for development (localhost:8000)

2. **Updated `docusaurus.config.ts`** to expose the backend URL via `customFields.backendUrl` with fallback to localhost

3. **Modified `ChatBubble.tsx`** to:
   - Import and use `useDocusaurusContext` hook
   - Retrieve dynamic `backendUrl` from `siteConfig.customFields`
   - Replace hardcoded `http://127.0.0.1:8000/ask` with `${backendUrl}/ask`
   - Update error messages to display the actual backend URL being used

4. **Verified CORS configuration** - Backend already has CORS enabled with `allow_origins=["*"]` (should be restricted to production domain for actual deployment)

5. **Error handling** - The UI already has loading states ("AI is thinking..." with pulsing animation) and error messages that now show the actual backend URL being contacted

## Outcome

- ✅ Impact: Frontend can now be deployed to production with configurable backend URL via environment variables
- 🧪 Tests: Manual verification needed - test by changing SITE_BACKEND_URL and verifying API calls go to the correct endpoint
- 📁 Files: Created 1 file (doc-book/.env.local), modified 2 files (docusaurus.config.ts, ChatBubble.tsx)
- 🔁 Next prompts: Configure backend CORS for production domain restriction; Set up production environment variables for deployment platform
- 🧠 Reflection: Implementation follows Docusaurus best practices using customFields and useDocusaurusContext hook. Error messages now provide more context by showing actual backend URL.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (manual verification required)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Restrict backend CORS to specific production domains instead of wildcard (*)
