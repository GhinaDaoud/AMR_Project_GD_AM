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

window.Thumb = function Thumb({ index, name }) {
  const n = (name || '').toLowerCase().replace(/\s+/g, '_');

  const ROLES = {
    supermarket:  { emoji: '🛒', color: '#10b981' },
    restaurant:   { emoji: '🍽️', color: '#f43f5e' },
    pharmacy:     { emoji: '💊', color: '#06b6d4' },
    fire_station: { emoji: '🚒', color: '#f97316' },
    firestation:  { emoji: '🚒', color: '#f97316' },
  };

  let entry;
  if (n.startsWith('house')) {
    // house_1 … house_5 — each gets a distinct warm shade so they're easy to tell apart
    const houseColors = ['#f59e0b', '#fb923c', '#a78bfa', '#34d399', '#60a5fa'];
    const houseNum = parseInt(n.replace(/\D/g, ''), 10) || 1;
    entry = { emoji: '🏠', color: houseColors[(houseNum - 1) % houseColors.length] };
  } else {
    const fallback = ['#6366f1', '#8b5cf6', '#84cc16'];
    entry = ROLES[n] || { emoji: '📍', color: fallback[(index - 1) % fallback.length] };
  }

  return React.createElement(
    'div',
    { className: 'lm-thumb', style: { background: entry.color + '20', border: `1.5px solid ${entry.color}40` } },
    entry.emoji
  );
};