# Design System: UI/UX Upgrade: The Physical AI Theme

**Feature**: 5-ui-ux-physical-ai-theme
**Date**: 2025-12-19
**Status**: Completed

## Overview

This document defines the design system elements, styling components, and visual patterns for the cyber-physical UI/UX theme upgrade.

## Color System

### Primary Palette
- **DeepSlate**: `#2a2f35` - Primary background color
- **Charcoal**: `#1a1d21` - Secondary/darker background
- **ElectricBlue**: `#00eeff` - Primary accent/highlight color
- **NeonGreen**: `#39ff14` - Secondary accent color
- **LightText**: `#e6e6e6` - Primary text color
- **MediumText**: `#a6a6a6` - Secondary text color
- **DarkText**: `#4d4d4d` - Disabled/tertiary text color

### Semantic Colors
- **Warning**: `#ff6b6b` - For warning admonitions (high contrast)
- **Tip**: `#4ecdc4` - For tip/admonition boxes
- **Success**: `#51cf66` - For success states
- **Danger**: `#ff6b6b` - For error states
- **Info**: `#339af0` - For informational content

## Typography System

### Font Stack
- **Body Text**: `Inter, -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, Oxygen, Ubuntu, Cantarell, Open Sans, Helvetica Neue, sans-serif`
- **Code Text**: `JetBrains Mono, SFMono-Regular, Consolas, Liberation Mono, Menlo, monospace`
- **Heading Text**: Same as body text but with increased weight and size hierarchy

### Typography Scale
- **H1**: 2.5rem (40px), font-weight: 700
- **H2**: 2rem (32px), font-weight: 600
- **H3**: 1.75rem (28px), font-weight: 600
- **H4**: 1.5rem (24px), font-weight: 600
- **Body Large**: 1.125rem (18px), font-weight: 400
- **Body Regular**: 1rem (16px), font-weight: 400
- **Body Small**: 0.875rem (14px), font-weight: 400
- **Code**: 0.875rem (14px), font-weight: 400

## Spacing System

### Base Unit
- Base unit: 8px
- Scale: 0.5x, 1x, 1.5x, 2x, 3x, 4x, 6x, 8x
- Values: 4px, 8px, 12px, 16px, 24px, 32px, 48px, 64px

## Component Styling

### Button Component
- **Primary**: Electric Blue background, white text, neon glow on hover
- **Secondary**: Transparent with electric blue border, electric blue text
- **Size Variants**: Small (12px padding), Medium (16px padding), Large (20px padding)
- **States**: Default, Hover (neon glow), Active, Disabled

### Admonition Components
- **Info Admonition**:
  - Background: rgba(0, 238, 255, 0.1)
  - Border: 1px solid rgba(0, 238, 255, 0.3)
  - Icon: Info circle in electric blue
- **Tip Admonition**:
  - Background: rgba(57, 255, 20, 0.1)
  - Border: 1px solid rgba(57, 255, 20, 0.3)
  - Icon: Lightbulb in neon green
- **Warning Admonition**:
  - Background: rgba(255, 107, 107, 0.1)
  - Border: 1px solid rgba(255, 107, 107, 0.3)
  - Icon: Triangle in warning red
- **Danger Admonition**:
  - Background: rgba(255, 107, 107, 0.15)
  - Border: 1px solid rgba(255, 107, 107, 0.4)
  - Icon: Exclamation in danger red

### Code Block Styling
- **Background**: `#1e1e1e` (dark gray)
- **Keyword**: `#00eeff` (electric blue)
- **String**: `#39ff14` (neon green)
- **Comment**: `#8c8c8c` (medium gray)
- **Function**: `#00eeff` (electric blue)
- **Number**: `#ff9d00` (orange)
- **Punctuation**: `#ffffff` (white)

### Navigation Elements
- **Navbar**: Glassmorphism effect with backdrop-filter
  - Background: `rgba(26, 29, 33, 0.85)`
  - Border: `1px solid rgba(0, 238, 255, 0.1)`
  - Blur: `backdrop-filter: blur(10px)`
- **Sidebar**: Custom scrollbar with electric blue thumb
  - Width: 6px
  - Background: `rgba(255, 255, 255, 0.1)`
  - Thumb: `#00eeff`
- **Active Indicator**: Electric blue left border (3px)

### Card/Feature Block Components
- **Background**: `rgba(255, 255, 255, 0.05)` (light overlay on dark)
- **Hover Effect**: Subtle glow with electric blue
- **Border Radius**: 8px
- **Padding**: 24px
- **Shadow**: `0 4px 6px rgba(0, 0, 0, 0.1)`

## Layout System

### Grid System
- **Breakpoints**:
  - Mobile: <768px (1 column)
  - Tablet: 768px-1024px (2 columns)
  - Desktop: >1024px (3 columns)
- **Gutter**: 24px between columns
- **Container**: Max-width 1200px, centered

### Responsive Typography
- **H1**: 40px desktop, 32px tablet, 28px mobile
- **H2**: 32px desktop, 28px tablet, 24px mobile
- **Body**: 16px desktop, 16px tablet, 15px mobile

## Animation System

### Transition Standards
- **Duration**: 200ms for hover states
- **Easing**: `ease-in-out`
- **Properties**: color, background-color, transform, box-shadow

### Micro-interactions
- **Button Hover**: Scale 1.02 with neon glow
- **Card Hover**: Lift effect with subtle shadow increase
- **Navigation**: Smooth highlight transition
- **Theme Switch**: 300ms fade transition between themes

## Accessibility Standards

### Focus States
- **Outline**: 2px solid `#00eeff` with 2px offset
- **Background**: Subtle darkening for better visibility
- **Duration**: 150ms transition for smooth focus change

### Contrast Ratios
- **Text on Background**: Minimum 7:1 ratio (AA compliance)
- **Interactive Elements**: Minimum 4.5:1 ratio
- **Large Text**: Minimum 3:1 ratio