# Quickstart: UI/UX Upgrade: The Physical AI Theme

**Feature**: 5-ui-ux-physical-ai-theme
**Date**: 2025-12-19

## Overview

This quickstart guide provides instructions for implementing the cyber-physical UI/UX theme with dark mode aesthetic for the Docusaurus documentation site.

## Prerequisites

- Node.js 18+ and npm/yarn
- Docusaurus 3.x project already set up
- Access to project source files
- Basic understanding of CSS and React

## Setup Steps

### 1. Update Configuration

First, update the Docusaurus configuration to support the new theme:

```javascript
// docusaurus.config.js
module.exports = {
  // ... existing config
  themeConfig: {
    // ... existing theme config
    colorMode: {
      defaultMode: 'dark',  // Set dark mode as default
      disableSwitch: false, // Allow users to switch to light mode
      respectPrefersColorScheme: true, // Respect system preference
    },
    // ... rest of config
  },

  // Add font loading
  stylesheets: [
    'https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&display=swap',
    'https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;500;600&display=swap',
  ],
};
```

### 2. Create Custom CSS Variables

Create or update `src/css/custom.css` with the cyber-physical theme:

```css
/* src/css/custom.css */
:root {
  /* Cyber-Physical Color Palette */
  --ifm-color-primary: #00eeff; /* Electric Blue */
  --ifm-color-primary-dark: #00d4e6; /* Darker Electric Blue */
  --ifm-color-primary-darker: #00c8d9; /* Even Darker */
  --ifm-color-primary-darkest: #00a6b3; /* Darkest */
  --ifm-color-primary-light: #1affff; /* Lighter Electric Blue */
  --ifm-color-primary-lighter: #2affff; /* Even Lighter */
  --ifm-color-primary-lightest: #4dffff; /* Lightest */

  /* Background Colors */
  --ifm-background-color: #2a2f35; /* Deep Slate */
  --ifm-background-surface-color: #1a1d21; /* Charcoal */

  /* Text Colors */
  --ifm-font-color-base: #e6e6e6; /* Light Text */
  --ifm-font-color-secondary: #a6a6a6; /* Medium Text */
  --ifm-font-color-third: #8c8c8c; /* Dark Text */

  /* Code Colors */
  --ifm-code-background: #1e1e1e; /* Dark Code Background */
  --ifm-code-color: #e6e6e6; /* Code Text Color */

  /* Border Colors */
  --ifm-border-color: #4d4d4d; /* Medium Gray Border */
  --ifm-global-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);

  /* Typography */
  --ifm-font-family-base: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
  --ifm-font-family-monospace: 'JetBrains Mono', 'SFMono-Regular', 'Consolas', 'Liberation Mono', 'Menlo', monospace;
}

/* Dark Mode Specifics */
html[data-theme='dark'] {
  --ifm-background-color: #1a1d21; /* Charcoal */
  --ifm-background-surface-color: #2a2f35; /* Deep Slate */
}

/* Light Mode (fallback) */
html[data-theme='light'] {
  --ifm-background-color: #ffffff; /* White */
  --ifm-background-surface-color: #f8f9fa; /* Light Gray */
  --ifm-font-color-base: #212529; /* Dark Text */
}
```

### 3. Implement Glassmorphism Navbar

Add glassmorphism effect to the navbar in `src/css/custom.css`:

```css
/* Glassmorphism Navbar */
.navbar {
  background-color: rgba(26, 29, 33, 0.85) !important;
  backdrop-filter: blur(10px);
  border-bottom: 1px solid rgba(0, 238, 255, 0.1);
}

.navbar-sidebar {
  background-color: #1a1d21;
}
```

### 4. Customize Code Block Syntax Highlighting

Add custom Prism theme to match the dark aesthetic:

```css
/* Custom Prism Theme */
code[class*="language-"],
pre[class*="language-"] {
  color: #e6e6e6;
  background: #1e1e1e;
  text-shadow: none;
  font-family: var(--ifm-font-family-monospace);
  font-size: var(--ifm-code-font-size);
  line-height: 1.5;
  direction: ltr;
  text-align: left;
  white-space: pre;
  word-spacing: normal;
  word-break: normal;
  word-wrap: normal;
  tab-size: 4;
  hyphens: none;
}

.token.comment,
.token.prolog,
.token.doctype,
.token.cdata {
  color: #8c8c8c;
}

.token.punctuation {
  color: #ffffff;
}

.token.namespace {
  opacity: 0.7;
}

.token.property,
.token.tag,
.token.boolean,
.token.number,
.token.constant,
.token.symbol,
.token.deleted {
  color: #ff9d00;
}

.token.selector,
.token.attr-name,
.token.string,
.token.char,
.token.builtin,
.token.inserted {
  color: #39ff14; /* Neon Green */
}

.token.operator,
.token.entity,
.token.url,
.language-css .token.string,
.style .token.string {
  color: #00eeff; /* Electric Blue */
}

.token.atrule,
.token.attr-value,
.token.keyword {
  color: #00eeff; /* Electric Blue */
}

.token.function,
.token.class-name {
  color: #00eeff; /* Electric Blue */
}

.token.regex,
.token.important,
.token.variable {
  color: #ffaa00;
}

.token.important,
.token.bold {
  font-weight: bold;
}
.token.italic {
  font-style: italic;
}

.token.entity {
  cursor: help;
}
```

### 5. Customize Admonitions

Add industrial-style admonitions to match the technical theme:

```css
/* Custom Admonition Styles */
.theme-admonition {
  border-left: 3px solid var(--ifm-color-primary);
}

/* Warning Admonition - Industrial Style */
.alert--warning {
  background-color: rgba(255, 107, 107, 0.1);
  border: 1px solid rgba(255, 107, 107, 0.3);
  border-left: 3px solid #ff6b6b;
}

/* Tip Admonition - Technical Style */
.alert--tip {
  background-color: rgba(57, 255, 20, 0.1);
  border: 1px solid rgba(57, 255, 20, 0.3);
  border-left: 3px solid #39ff14;
}

/* Info Admonition */
.alert--info {
  background-color: rgba(51, 154, 240, 0.1);
  border: 1px solid rgba(51, 154, 240, 0.3);
  border-left: 3px solid #339af0;
}

/* Danger Admonition */
.alert--danger {
  background-color: rgba(255, 107, 107, 0.15);
  border: 1px solid rgba(255, 107, 107, 0.4);
  border-left: 3px solid #ff6b6b;
}
```

### 6. Customize Scrollbars

Add custom scrollbar styling for the sidebar:

```css
/* Custom Scrollbar */
::-webkit-scrollbar {
  width: 6px;
  height: 6px;
}

::-webkit-scrollbar-track {
  background: rgba(255, 255, 255, 0.1);
  border-radius: 3px;
}

::-webkit-scrollbar-thumb {
  background: #00eeff;
  border-radius: 3px;
}

::-webkit-scrollbar-thumb:hover {
  background: #00d4e6;
}
```

### 7. Update Landing Page

Create or update the landing page with the new design:

```javascript
// src/pages/index.js
import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">Bridging Digital Brains & Physical Bodies</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start the Course
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Hello from ${siteConfig.title}`}
      description="Description will go into a meta tag in <head />">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
```

### 8. Add SVG Icons

Place your custom SVG icons in the `static/img` directory and reference them in your components.

## Testing & Validation

### 1. Accessibility Testing
- Run Lighthouse audit to ensure accessibility score >90
- Verify color contrast ratios meet WCAG AA standards
- Test keyboard navigation and focus indicators

### 2. Performance Testing
- Ensure page load times remain under 3 seconds
- Verify font loading doesn't block rendering
- Test on various devices and browsers

### 3. Responsive Testing
- Test on mobile, tablet, and desktop
- Verify layout adapts appropriately
- Ensure touch targets are appropriately sized

## Next Steps

1. Run `npm run build` to build the site with the new theme
2. Run `npm run serve` to test the production build locally
3. Deploy to your hosting platform
4. Monitor user feedback and analytics
5. Iterate based on user experience data