/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    './src/**/*.{js,jsx,ts,tsx}',
    './docs/**/*.{md,mdx}',
    './blog/**/*.{md,mdx}',
    './pages/**/*.{md,mdx}',
    './docusaurus.config.ts',
  ],
  theme: {
    extend: {
      colors: {
        'bot-dark': '#0f172a', // Deep Slate
        'bot-cyan': '#22d3ee', // Neon Accent
        'bot-blue': '#2563eb', // Primary Blue
      },
    },
  },
  plugins: [],
}