# CSS Variables Contract: Cyber-Physical Theme

**Feature**: 5-ui-ux-physical-ai-theme
**Contract Version**: 1.0
**Date**: 2025-12-19

## Overview

This contract defines the CSS custom properties (variables) that implement the cyber-physical theme with deep slate and electric blue aesthetic. These variables override Docusaurus/Infima default styles.

## Color Variables

### Primary Colors
```css
--ifm-color-primary: #00eeff; /* Electric Blue - main accent */
--ifm-color-primary-dark: #00d4e6; /* Darker Electric Blue */
--ifm-color-primary-darker: #00c8d9; /* Even Darker */
--ifm-color-primary-darkest: #00a6b3; /* Darkest */
--ifm-color-primary-light: #1affff; /* Lighter Electric Blue */
--ifm-color-primary-lighter: #2affff; /* Even Lighter */
--ifm-color-primary-lightest: #4dffff; /* Lightest */
```

### Background Colors
```css
--ifm-background-color: #1a1d21; /* Charcoal - main background */
--ifm-background-surface-color: #2a2f35; /* Deep Slate - surface elements */
--ifm-code-background: #1e1e1e; /* Dark Code Background */
```

### Text Colors
```css
--ifm-font-color-base: #e6e6e6; /* Light Text - primary content */
--ifm-font-color-secondary: #a6a6a6; /* Medium Text - secondary content */
--ifm-font-color-third: #8c8c8c; /* Dark Text - muted content */
```

### Semantic Colors
```css
--ifm-color-success: #51cf66; /* Success state */
--ifm-color-info: #339af0; /* Info state */
--ifm-color-warning: #fcc419; /* Warning state */
--ifm-color-danger: #ff6b6b; /* Danger state */
```

## Typography Variables

### Font Families
```css
--ifm-font-family-base: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
--ifm-font-family-monospace: 'JetBrains Mono', 'SFMono-Regular', 'Consolas', 'Liberation Mono', 'Menlo', monospace;
```

### Font Sizes
```css
--ifm-h1-font-size: 2.5rem; /* 40px */
--ifm-h2-font-size: 2rem; /* 32px */
--ifm-h3-font-size: 1.75rem; /* 28px */
--ifm-h4-font-size: 1.5rem; /* 24px */
--ifm-font-size-base: 1rem; /* 16px - body text */
--ifm-code-font-size: 0.875rem; /* 14px - code text */
```

### Font Weights
```css
--ifm-font-weight-semibold: 600; /* For headings */
--ifm-font-weight-bold: 700; /* For strong emphasis */
```

## Spacing Variables

### Base Spacing Unit
```css
--ifm-global-spacing: 1rem; /* 16px base unit */
--ifm-spacing-vertical: var(--ifm-global-spacing);
--ifm-spacing-horizontal: var(--ifm-global-spacing);
```

### Component-Specific Spacing
```css
--ifm-global-radius: 8px; /* Border radius for components */
--ifm-global-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); /* Standard shadow */
```

## Component-Specific Variables

### Navbar Variables
```css
--ifm-navbar-background-color: rgba(26, 29, 33, 0.85); /* Glassmorphism effect */
--ifm-navbar-shadow: 0 1px 0 rgba(0, 238, 255, 0.1); /* Subtle bottom border */
```

### Sidebar Variables
```css
--ifm-menu-color-background-active: rgba(0, 238, 255, 0.1); /* Active item highlight */
--ifm-menu-color: var(--ifm-font-color-base); /* Menu text color */
```

### Admonition Variables
```css
--ifm-alert-background-color: var(--ifm-background-surface-color); /* Base alert background */
--ifm-alert-border-left-width: 3px; /* Border width for alerts */
```

### Code Block Variables
```css
--ifm-pre-background: var(--ifm-code-background); /* Pre element background */
--ifm-code-border-radius: 6px; /* Code block border radius */
--ifm-pre-border-radius: var(--ifm-code-border-radius); /* Pre border radius */
```

## Responsive Variables

### Breakpoints
```css
--ifm-container-width: 1200px; /* Max container width */
--ifm-container-width-xl: 1400px; /* Extra large container width */
```

## Theme-Specific Variables

### Dark Mode Overrides
```css
html[data-theme='dark'] {
  --ifm-background-color: #1a1d21;
  --ifm-background-surface-color: #2a2f35;
  --ifm-font-color-base: #e6e6e6;
}
```

### Light Mode Overrides
```css
html[data-theme='light'] {
  --ifm-background-color: #ffffff;
  --ifm-background-surface-color: #f8f9fa;
  --ifm-font-color-base: #212529;
}
```

## Usage Guidelines

1. **Consistency**: Always use CSS variables instead of hardcoded values
2. **Accessibility**: Ensure all color combinations meet WCAG AA contrast requirements
3. **Performance**: Minimize the number of custom variables to reduce CSS bundle size
4. **Maintainability**: Group related variables together and use descriptive names
5. **Compatibility**: These variables override Docusaurus/Infima defaults while maintaining upgrade compatibility

## Validation Requirements

- All color contrast ratios must meet WCAG AA standards (minimum 4.5:1 for normal text)
- Variables must work across all supported browsers (Chrome, Firefox, Safari, Edge)
- Theme switching must work seamlessly with Docusaurus built-in functionality
- Performance impact should be minimal (no more than 5KB CSS increase)