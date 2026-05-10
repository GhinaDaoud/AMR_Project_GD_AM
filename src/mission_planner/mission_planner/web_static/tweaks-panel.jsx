// Tweaks / settings panel

window.TweaksPanel = function TweaksPanel({ show, onClose, showCoords, setShowCoords }) {
  if (!show) return null;

  return React.createElement(
    'div',
    { className: 'tweaks-panel' },
    React.createElement('h3', null, '⚙  Settings'),

    React.createElement(
      'div',
      { className: 'tweak-row' },
      React.createElement('label', null, 'Show coordinates'),
      React.createElement(
        'button',
        {
          className: `toggle ${showCoords ? 'on' : 'off'}`,
          onClick: () => setShowCoords(!showCoords),
        }
      )
    ),

    React.createElement(
      'button',
      {
        className: 'ctrl-btn',
        style: { marginTop: 8 },
        onClick: onClose,
      },
      'Close'
    )
  );
};