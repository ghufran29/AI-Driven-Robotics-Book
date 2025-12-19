// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics: Bridging Digital & Physical Worlds',
  tagline: 'Comprehensive Curriculum for Modern Robotics Development',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'facebook', // Usually your GitHub org/user name.
  projectName: 'docusaurus', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Add Google Fonts with optimization
  stylesheets: [
    {
      href: 'https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap',
      type: 'text/css',
      rel: 'stylesheet',
      crossOrigin: 'anonymous',
    },
    {
      href: 'https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600&display=swap',
      type: 'text/css',
      rel: 'stylesheet',
      crossOrigin: 'anonymous',
    },
  ],
  scripts: [
    {
      src: 'https://polyfill.io/v3/polyfill.min.js?features=es6',
      async: true,
    },
  ],
  headTags: [
    // Preconnect to Google Fonts for performance
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossOrigin: 'anonymous',
      },
    },
  ],

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-organization/ai-book/tree/main/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        defaultMode: 'dark',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'AI Robotics Book',
        logo: {
          alt: 'AI Robotics Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Curriculum',
          },
          {
            href: 'https://github.com/facebook/docusaurus',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Curriculum',
            items: [
              {
                label: 'Introduction to Physical AI',
                to: '/docs/intro',
              },
              {
                label: 'ROS 2 Framework',
                to: '/docs/module-01-ros2-fundamentals/foundational-concepts',
              },
              {
                label: 'Simulation Environments',
                to: '/docs/module-02-digital-twin/laws-of-physics-gazebo',
              },
              {
                label: 'AI-Robot Brain',
                to: '/docs/module-03-isaac-brain/omniverse-isaac',
              },
            ],
          },
          {
            title: 'Research Labs',
            items: [
              {
                label: 'Robotics Institute',
                href: 'https://example.com/robotics',
              },
              {
                label: 'AI & Machine Learning Lab',
                href: 'https://example.com/ai-lab',
              },
              {
                label: 'Humanoid Robotics Lab',
                href: 'https://example.com/humanoid',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-organization/ai-book',
              },
              {
                label: 'Research Papers',
                href: 'https://example.com/papers',
              },
              {
                label: 'Quick Start',
                to: '/docs/module-01-ros2-fundamentals/quickstart',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AI Robotics Education - Advancing Humanoid Robotics Research. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
