import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';

const config: Config = {
  title: 'Humanoid Academy',
  tagline: 'From ROS 2 foundations to AI-driven humanoid systems',

  url: 'https://your-github-username.github.io',
  baseUrl: '/',

  favicon: 'img/favicon.ico',

  organizationName: 'your-github-username',
  projectName: 'doc-book',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          path: 'docs',
          routeBasePath: '/',
          sidebarPath: require.resolve('./sidebars.ts'),
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Humanoid Academy',
      style: 'primary',
      items: [
        {
          to: '/docs/intro',
          label: 'Library',
          position: 'left',
        },
        {
          href: '#',
          label: 'Sign In',
          position: 'right',
        },
        {
          href: '#',
          label: 'Sign Up',
          position: 'right',
          className: 'button button--primary',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: '[ Project Info ]',
          items: [
            {
              label: 'Mission',
              to: '/',
            },
            {
              label: 'About the Author (Asad)',
              to: '/',
            },
          ],
        },
        {
          title: '[ Modules ]',
          items: [
            {
              label: 'Module 1 (Nervous System)',
              to: '/docs/module-1/chapter-1-the-ros-2-communication-backbone',
            },
            {
              label: 'Module 2 (Digital Twin)',
              to: '/docs/module-2/chapter-1-gazebo-physics-simulation',
            },
            {
              label: 'Module 3 (AI Brain)',
              to: '/docs/module-3/chapter-1-photorealistic-intelligence',
            },
            {
              label: 'Module 4 (VLA Capstone)',
              to: '/docs/module-4/chapter-1-voice-to-action',
            },
          ],
        },
        {
          title: '[ Tools ]',
          items: [
            {
              label: 'ROS 2 Humble',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'NVIDIA Isaac Sim',
              href: 'https://developer.nvidia.com/isaac-sim',
            },
            {
              label: 'Unity',
              href: 'https://unity.com/',
            },
            {
              label: 'OpenAI Whisper',
              href: 'https://platform.openai.com/docs/guides/speech-to-text',
            },
          ],
        },
        {
          title: '[ Community ]',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/',
            },
            {
              label: 'LinkedIn',
              href: 'https://linkedin.com/',
            },
            {
              label: 'Discord',
              href: 'https://discord.com/',
            },
          ],
        },
      ],
      copyright: `© 2025 doc-book-By-Asad | Built with Docusaurus | Humanoid Academy`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  },
};

export default config;
