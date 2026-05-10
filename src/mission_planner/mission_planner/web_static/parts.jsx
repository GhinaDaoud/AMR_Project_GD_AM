// Shared UI primitives — no hardcoded landmark data

window.Icon = function Icon({ name, size = 16 }) {
  const icons = {
    robot: '🤖', home: '🏠', plus: '+', trash: '✕',
    up: '▲', down: '▼', go: '▶', check: '✓', x: '✗',
    spin: '↻', settings: '⚙', map: '📍', clock: '⏱',
    arrow: '→',
  };
  return React.createElement(
    'span',
    { style: { fontSize: size, lineHeight: 1 } },
    icons[name] || name
  );
};

window.Thumb = function Thumb({ index }) {
  const colors = [
    '#6366f1', '#10b981', '#f59e0b', '#f43f5e',
    '#8b5cf6', '#06b6d4', '#84cc16', '#f97316',
  ];
  const emojis = ['📦', '🔵', '🟡', '🔴', '🟣', '🔷', '🟢', '🟠'];
  const color = colors[(index - 1) % colors.length];
  const emoji = emojis[(index - 1) % emojis.length];
  return React.createElement(
    'div',
    { className: 'lm-thumb', style: { background: color + '20', border: `1.5px solid ${color}40` } },
    emoji
  );
};