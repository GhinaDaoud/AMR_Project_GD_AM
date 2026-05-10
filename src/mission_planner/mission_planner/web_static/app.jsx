// Main app — connects to Flask/ROS2 backend

const { useState, useEffect, useRef, useCallback } = React;

// ── helpers ────────────────────────────────────────────────────────────

function cleanName(name) {
  // Strip trailing coordinate hints like "(0, -4)"
  return name.replace(/\s*\([^)]*\)\s*$/, '').trim();
}

function parseStatus(raw) {
  // Returns { kind, step, total, label } or null
  if (raw === 'DOCKED') return { kind: 'DOCKED', step: 0, total: 0, label: 'Docking Station' };
  const parts = raw.split(':', 3);
  if (parts.length < 3) return null;
  const [kind, stepStr, rest] = parts;
  if (!['GOING', 'AT', 'FAILED'].includes(kind)) return null;
  const colonIdx = raw.indexOf(':', raw.indexOf(':', raw.indexOf(':') + 1) + 1);
  const label = colonIdx !== -1 ? raw.slice(colonIdx + 1) : '';
  return { kind, step: parseInt(stepStr, 10), total: parseInt(rest, 10), label };
}

// ── step state icon ────────────────────────────────────────────────────

function stepClass(state) {
  if (state === 'going')  return 'step-row state-going';
  if (state === 'at')     return 'step-row state-at';
  if (state === 'failed') return 'step-row state-failed';
  return 'step-row state-pending';
}

function stepIcon(state) {
  if (state === 'going')  return '▶';
  if (state === 'at')     return '✓';
  if (state === 'failed') return '✗';
  return '';
}

// ── status bar data ────────────────────────────────────────────────────

function statusInfo(status) {
  if (!status) return { cls: 'idle', icon: '💤', label: 'IDLE', msg: 'Ready — build a path and press Start' };
  if (status.kind === 'DOCKED') return { cls: 'docked', icon: '🏠', label: 'DOCKED', msg: 'Mission complete — robot is docked' };
  if (status.kind === 'GOING') {
    const isHome = status.step === status.total;
    return {
      cls: 'going',
      icon: '▶',
      label: 'GOING',
      msg: isHome
        ? `Returning to dock  (step ${status.step} of ${status.total})`
        : `Going to "${cleanName(status.label)}"  (step ${status.step} of ${status.total})`,
    };
  }
  if (status.kind === 'AT') return {
    cls: 'at', icon: '✓', label: 'AT',
    msg: `At "${cleanName(status.label)}"  (step ${status.step} of ${status.total}) — waiting 2 s`,
  };
  if (status.kind === 'FAILED') return {
    cls: 'failed', icon: '✗', label: 'FAILED',
    msg: `Failed at "${cleanName(status.label)}"  (step ${status.step} of ${status.total})`,
  };
  return { cls: 'idle', icon: '💤', label: 'IDLE', msg: '' };
}

// ── main component ─────────────────────────────────────────────────────

function App() {
  const [landmarks, setLandmarks]     = useState([]);
  const [queue, setQueue]             = useState([]);        // [{id, name, displayName}]
  const [stepStates, setStepStates]   = useState([]);        // 'pending'|'going'|'at'|'failed'
  const [status, setStatus]           = useState(null);      // parsed status object
  const [connected, setConnected]     = useState(false);
  const [showCoords, setShowCoords]   = useState(false);
  const [showTweaks, setShowTweaks]   = useState(false);
  const [selected, setSelected]       = useState(null);      // selected step index
  const eventSourceRef = useRef(null);

  // ── fetch landmarks on mount ──────────────────────────────────────
  useEffect(() => {
    fetch('/api/landmarks')
      .then(r => r.json())
      .then(data => setLandmarks(data))
      .catch(err => console.error('Failed to load landmarks:', err));
  }, []);

  // ── SSE subscription ──────────────────────────────────────────────
  useEffect(() => {
    const es = new EventSource('/api/mission/status');
    eventSourceRef.current = es;

    es.onopen = () => setConnected(true);
    es.onerror = () => setConnected(false);

    es.onmessage = (ev) => {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.type === 'connected') { setConnected(true); return; }
        if (msg.type === 'ping') return;
        if (msg.type === 'status') {
          const parsed = parseStatus(msg.value);
          if (!parsed) return;
          setStatus(parsed);

          // Update per-step state colors
          setStepStates(prev => {
            const next = [...prev];
            if (parsed.kind === 'DOCKED') {
              return next.map(() => 'at');  // all done
            }
            const lbIdx = parsed.step - 1;
            if (parsed.kind === 'GOING') {
              // mark previous steps done, current going
              return next.map((s, i) => {
                if (i < lbIdx - 1) return 'at';
                if (i === lbIdx) return 'going';  // wait for AT to confirm
                return s;
              });
            }
            if (parsed.kind === 'AT') {
              return next.map((s, i) => (i === lbIdx ? 'at' : s));
            }
            if (parsed.kind === 'FAILED') {
              return next.map((s, i) => (i === lbIdx ? 'failed' : s));
            }
            return next;
          });
        }
      } catch (e) { /* ignore */ }
    };

    return () => es.close();
  }, []);

  // ── queue operations ──────────────────────────────────────────────

  const addLandmark = useCallback((lm) => {
    setQueue(prev => [...prev, { id: lm.id, name: lm.name, displayName: cleanName(lm.name) }]);
    setStepStates(prev => [...prev, 'pending']);
  }, []);

  const removeStep = useCallback((idx) => {
    setQueue(prev => prev.filter((_, i) => i !== idx));
    setStepStates(prev => prev.filter((_, i) => i !== idx));
    setSelected(null);
  }, []);

  const moveUp = useCallback((idx) => {
    if (idx === 0) return;
    setQueue(prev => { const a = [...prev]; [a[idx-1], a[idx]] = [a[idx], a[idx-1]]; return a; });
    setStepStates(prev => { const a = [...prev]; [a[idx-1], a[idx]] = [a[idx], a[idx-1]]; return a; });
    setSelected(idx - 1);
  }, []);

  const moveDown = useCallback((idx) => {
    setQueue(prev => {
      if (idx >= prev.length - 1) return prev;
      const a = [...prev]; [a[idx], a[idx+1]] = [a[idx+1], a[idx]]; return a;
    });
    setStepStates(prev => {
      if (idx >= prev.length - 1) return prev;
      const a = [...prev]; [a[idx], a[idx+1]] = [a[idx+1], a[idx]]; return a;
    });
    setSelected(idx + 1);
  }, []);

  const clearQueue = useCallback(() => {
    setQueue([]);
    setStepStates([]);
    setSelected(null);
    setStatus(null);
  }, []);

  const startMission = useCallback(() => {
    if (queue.length === 0) return;
    const sequence = queue.map(item => item.id).join(',');
    // Reset states
    setStepStates(queue.map(() => 'pending'));
    setStatus(null);
    fetch('/api/mission/start', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ sequence }),
    }).catch(err => console.error('Failed to start mission:', err));
  }, [queue]);

  // ── status bar ─────────────────────────────────────────────────────
  const si = statusInfo(status);
  const progressPct = status && status.total > 0
    ? Math.round((status.step / status.total) * 100)
    : 0;

  // ── render ─────────────────────────────────────────────────────────
  return React.createElement(
    'div',
    { className: 'app-shell', onClick: () => setShowTweaks(false) },

    // ── top bar ──
    React.createElement(
      'header',
      { className: 'top-bar' },
      React.createElement('div', { className: 'top-bar-icon' }, '🤖'),
      React.createElement('h1', null, 'AMR Robot Mission Planner'),
      React.createElement(
        'span',
        { className: 'subtitle' },
        connected
          ? React.createElement('span', { style: { color: '#10b981' } }, '● Connected')
          : React.createElement('span', { style: { color: '#f43f5e' } }, '○ Disconnected')
      ),
      React.createElement(
        'button',
        {
          className: 'tweaks-btn',
          onClick: (e) => { e.stopPropagation(); setShowTweaks(v => !v); },
          title: 'Settings',
        },
        '⚙'
      )
    ),

    // ── tweaks panel ──
    React.createElement(TweaksPanel, {
      show: showTweaks,
      onClose: () => setShowTweaks(false),
      showCoords,
      setShowCoords,
    }),

    // ── main grid ──
    React.createElement(
      'main',
      { className: 'main-content' },

      // LEFT: landmarks
      React.createElement(
        'div',
        { className: 'panel' },
        React.createElement(
          'div',
          { className: 'panel-header' },
          React.createElement('h2', null, '📍  Landmarks'),
          React.createElement(
            'span',
            { className: 'count-badge' },
            `${landmarks.length} found`
          )
        ),
        React.createElement(
          'div',
          { className: 'panel-body scroll' },
          landmarks.length === 0
            ? React.createElement('div', { className: 'empty-state' }, 'Loading landmarks…')
            : landmarks.map(lm =>
                React.createElement(
                  'div',
                  { className: 'landmark-card', key: lm.id, onClick: () => addLandmark(lm) },
                  React.createElement(Thumb, { index: lm.id }),
                  React.createElement(
                    'div',
                    { className: 'lm-info' },
                    React.createElement('div', { className: 'lm-name' }, cleanName(lm.name)),
                    showCoords && React.createElement(
                      'div',
                      { className: 'lm-coords' },
                      `x: ${lm.x.toFixed(2)}, y: ${lm.y.toFixed(2)}`
                    )
                  ),
                  React.createElement('div', { className: 'add-btn', title: 'Add to path' }, '+')
                )
              )
        )
      ),

      // RIGHT: mission path
      React.createElement(
        'div',
        { className: 'panel', style: { display: 'flex', flexDirection: 'column' } },
        React.createElement(
          'div',
          { className: 'panel-header' },
          React.createElement('h2', null, '🗺  Mission Path'),
          queue.length > 0 && React.createElement(
            'span',
            { className: 'count-badge' },
            `${queue.length} stop${queue.length !== 1 ? 's' : ''}`
          )
        ),

        // dock start
        React.createElement(
          'div',
          { className: 'dock-row top' },
          React.createElement('span', { className: 'dock-icon' }, '🏠'),
          React.createElement('span', { className: 'dock-label' }, 'Docking Station'),
          React.createElement('span', { className: 'dock-sub' }, 'start')
        ),

        // step list
        React.createElement(
          'div',
          { className: 'step-list', style: { flex: 1, overflowY: 'auto', paddingTop: 8 } },
          queue.length === 0
            ? React.createElement(
                'div',
                { className: 'empty-state' },
                React.createElement('div', { className: 'big' }, '🗺'),
                'Click a landmark to add it to the mission'
              )
            : queue.map((item, idx) =>
                React.createElement(
                  'div',
                  {
                    key: idx,
                    className: stepClass(stepStates[idx] || 'pending') + (selected === idx ? ' selected' : ''),
                    onClick: () => setSelected(idx),
                    style: { cursor: 'pointer', outline: selected === idx ? '2px solid #6366f1' : 'none' },
                  },
                  React.createElement(
                    'div',
                    { className: 'step-num' },
                    stepStates[idx] === 'pending' ? idx + 1 : React.createElement('span', null, stepIcon(stepStates[idx]))
                  ),
                  React.createElement('div', { className: 'step-label' }, item.displayName),
                  React.createElement(
                    'button',
                    {
                      className: 'remove-btn',
                      onClick: (e) => { e.stopPropagation(); removeStep(idx); },
                      title: 'Remove step',
                    },
                    '✕'
                  )
                )
              )
        ),

        // reorder controls
        queue.length > 0 && React.createElement(
          'div',
          { className: 'step-controls' },
          React.createElement(
            'button',
            { className: 'ctrl-btn', onClick: () => selected !== null && moveUp(selected), disabled: selected === null || selected === 0 },
            '▲ Up'
          ),
          React.createElement(
            'button',
            { className: 'ctrl-btn', onClick: () => selected !== null && moveDown(selected), disabled: selected === null || selected === queue.length - 1 },
            '▼ Down'
          )
        ),

        // dock end
        React.createElement(
          'div',
          {
            className: 'dock-row bottom',
            style: status && status.kind === 'DOCKED'
              ? { background: '#065f46' }
              : {},
          },
          React.createElement('span', { className: 'dock-icon' }, '🏠'),
          React.createElement('span', { className: 'dock-label' }, 'Docking Station'),
          React.createElement('span', { className: 'dock-sub' }, 'end')
        ),

        // action buttons
        React.createElement(
          'div',
          { className: 'action-row' },
          React.createElement(
            'button',
            { className: 'btn-start', onClick: startMission, disabled: queue.length === 0 },
            '▶   START MISSION'
          ),
          React.createElement(
            'button',
            { className: 'btn-clear', onClick: clearQueue },
            '✕ Clear'
          )
        )
      )
    ),

    // ── status bar ──
    React.createElement(
      'footer',
      { className: 'status-bar' },
      React.createElement('span', { className: 'status-icon' }, si.icon),
      React.createElement('span', { className: `status-label ${si.cls}` }, si.label),
      React.createElement('span', { className: 'status-message' }, si.msg),
      status && status.total > 0 && React.createElement(
        React.Fragment,
        null,
        React.createElement(
          'span',
          { className: 'status-counter' },
          `${status.step} / ${status.total}`
        ),
        React.createElement(
          'div',
          { className: 'progress-wrap' },
          React.createElement(
            'div',
            { className: 'progress-bar', style: { width: `${progressPct}%` } }
          )
        )
      ),
      React.createElement(
        'span',
        { style: { fontSize: 11, color: 'var(--text3)', marginLeft: 'auto', whiteSpace: 'nowrap' } },
        'Created by Abdallah & Ghina'
      )
    )
  );
}

ReactDOM.createRoot(document.getElementById('root')).render(React.createElement(App));