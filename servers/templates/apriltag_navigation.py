from .base import render_template

_EXTRA_CSS = '''
.info-box {
    background: var(--bg-sidebar);
    border: 1px solid var(--border-color);
    border-radius: 6px;
    padding: 16px;
}
.info-box h2 { font-size: 16px; font-weight: 600; margin-bottom: 12px; }
.info-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 8px; }
.info-item { display: flex; justify-content: space-between; padding: 8px; background: var(--bg-darker); border-radius: 4px; }
.info-label { color: var(--text-secondary); font-size: 13px; }
.info-value { color: var(--accent-blue); font-family: monospace; font-size: 13px; font-weight: 600; }
.control-group { margin-bottom: 20px; }
.control-group:last-child { margin-bottom: 0; }
.control-group label { display: block; margin-bottom: 8px; font-size: 14px; font-weight: 600; }
.control-row { display: flex; align-items: center; gap: 12px; }
.value-display { min-width: 60px; text-align: right; font-family: monospace; font-size: 13px; color: var(--text-secondary); }
.hsv-section-title { font-size: 13px; font-weight: 600; color: var(--text-secondary); margin: 12px 0 8px; text-transform: uppercase; letter-spacing: 0.5px; }
.hsv-section-title.yellow { color: #f1c40f; }
.hsv-section-title.white  { color: #ecf0f1; }
.tag-list, .event-log {
    font-family: 'SFMono-Regular', Consolas, monospace;
    font-size: 11px;
    color: var(--text-secondary);
    background: var(--bg-sidebar);
    border-radius: 4px;
    padding: 8px;
    max-height: 130px;
    overflow-y: auto;
    line-height: 1.6;
}
'''

_CONTENT = '''
    <div class="container">
        <div class="video-section">
            <img src="{{ url_for('video') }}" class="stream" alt="AprilTag Navigation Stream">
        </div>

        <div class="controls-section">

            <!-- Sign Detection card -->
            <div class="card">
                <div class="card-header">Sign Detection</div>
                <div class="info-grid" style="margin-bottom:12px">
                    <div class="info-item"><span class="info-label">State</span><span class="info-value" id="nav-state">DRIVE</span></div>
                    <div class="info-item"><span class="info-label">Countdown</span><span class="info-value" id="nav-countdown">-</span></div>
                </div>
                <div class="hsv-section-title">Detected Tags</div>
                <div id="nav-tags" class="tag-list">none</div>
                <div class="hsv-section-title">Event Log</div>
                <div id="nav-log" class="event-log"></div>
            </div>

            <!-- Drive Control card -->
            <div class="card">
                <div class="card-header">Drive Control</div>
                <div style="display:flex;align-items:center;gap:16px;margin-bottom:12px">
                    <span id="run-indicator" style="display:inline-block;width:14px;height:14px;border-radius:50%;background:#e74c3c;flex-shrink:0"></span>
                    <span id="run-label" style="font-size:14px;font-weight:600;color:var(--text-secondary)">STOPPED — camera learning lane width</span>
                </div>
                <div style="display:flex;gap:10px">
                    <button id="btn-start" onclick="driveStart()" class="button success" style="flex:1">Start</button>
                    <button id="btn-stop"  onclick="driveStop()"  class="button" style="flex:1;background:var(--accent-orange,#e67e22)">Stop</button>
                </div>
            </div>

            <!-- Control Parameters card -->
            <div class="card">
                <div class="card-header">Control Parameters</div>

                <div class="control-group">
                    <label for="k_d">Lateral Gain (k_d)</label>
                    <div class="control-row">
                        <input type="range" id="k_d" class="slider" min="0" max="1" step="0.01" value="{{ config.p_gain }}">
                        <span class="value-display" id="k_d_value">{{ config.p_gain }}</span>
                    </div>
                </div>

                <div class="control-group">
                    <label for="k_phi">Heading Gain (k_phi)</label>
                    <div class="control-row">
                        <input type="range" id="k_phi" class="slider" min="0" max="2" step="0.01" value="{{ config.d_gain }}">
                        <span class="value-display" id="k_phi_value">{{ config.d_gain }}</span>
                    </div>
                </div>

                <div class="control-group">
                    <label for="const">Base Speed <span style="color:var(--text-muted);font-weight:400;font-size:12px">(PWM 0–1)</span></label>
                    <div class="control-row">
                        <input type="range" id="const" class="slider" min="0" max="1" step="0.01" value="{{ config.base_speed }}">
                        <span class="value-display" id="const_value">{{ config.base_speed }}</span>
                    </div>
                </div>

                <button onclick="updateConfig()" class="button success">Apply Changes</button>
                <button onclick="resetPosition()" class="button" style="margin-top:8px;background:var(--accent-orange,#e67e22)">Reset Position</button>
                <div id="config-status" class="status"></div>
            </div>

            <!-- HSV Calibration card -->
            <div class="card">
                <div class="card-header">HSV Color Calibration</div>

                <div class="hsv-section-title yellow">Yellow Line (left / dashed)</div>

                <div class="slider-group">
                    <div class="slider-label"><span>Hue Low</span><span style="color:var(--text-muted)">0-179</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yLowH" min="0" max="179" value="20" class="slider">
                        <input type="number" id="yLowH-input" min="0" max="179" value="20" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Hue High</span><span style="color:var(--text-muted)">0-179</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yHighH" min="0" max="179" value="40" class="slider">
                        <input type="number" id="yHighH-input" min="0" max="179" value="40" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Saturation Low</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yLowS" min="0" max="255" value="80" class="slider">
                        <input type="number" id="yLowS-input" min="0" max="255" value="80" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Saturation High</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yHighS" min="0" max="255" value="255" class="slider">
                        <input type="number" id="yHighS-input" min="0" max="255" value="255" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Value Low</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yLowV" min="0" max="255" value="100" class="slider">
                        <input type="number" id="yLowV-input" min="0" max="255" value="100" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Value High</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="yHighV" min="0" max="255" value="255" class="slider">
                        <input type="number" id="yHighV-input" min="0" max="255" value="255" class="input-box">
                    </div>
                </div>

                <div class="hsv-section-title white" style="margin-top:20px">White Line (right / solid)</div>

                <div class="slider-group">
                    <div class="slider-label"><span>Hue Low</span><span style="color:var(--text-muted)">0-179</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wLowH" min="0" max="179" value="0" class="slider">
                        <input type="number" id="wLowH-input" min="0" max="179" value="0" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Hue High</span><span style="color:var(--text-muted)">0-179</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wHighH" min="0" max="179" value="179" class="slider">
                        <input type="number" id="wHighH-input" min="0" max="179" value="179" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Saturation Low</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wLowS" min="0" max="255" value="0" class="slider">
                        <input type="number" id="wLowS-input" min="0" max="255" value="0" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Saturation High</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wHighS" min="0" max="255" value="40" class="slider">
                        <input type="number" id="wHighS-input" min="0" max="255" value="40" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Value Low</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wLowV" min="0" max="255" value="180" class="slider">
                        <input type="number" id="wLowV-input" min="0" max="255" value="180" class="input-box">
                    </div>
                </div>
                <div class="slider-group">
                    <div class="slider-label"><span>Value High</span><span style="color:var(--text-muted)">0-255</span></div>
                    <div class="slider-controls">
                        <input type="range" id="wHighV" min="0" max="255" value="255" class="slider">
                        <input type="number" id="wHighV-input" min="0" max="255" value="255" class="input-box">
                    </div>
                </div>

                <div id="hsv-status" class="status"></div>
            </div>

        </div>
    </div>
'''

_JS = '''
    const NAV_STATE_COLORS = {
        DRIVE: 'var(--accent-green)',
        STOPPED: 'var(--accent-red)',
        YIELDING: '#f1c40f',
        DUCK_WAIT: 'var(--accent-orange)',
    };

    function refreshNavStatus() {
        fetch('/status').then(r => r.json()).then(d => {
            if (d.status !== 'active') return;

            const stateEl = document.getElementById('nav-state');
            stateEl.textContent = d.state;
            stateEl.style.color = NAV_STATE_COLORS[d.state] || 'var(--accent-blue)';

            const countdown = document.getElementById('nav-countdown');
            countdown.textContent = d.state_remaining > 0 ? d.state_remaining.toFixed(1) + 's' : '-';

            const tagsEl = document.getElementById('nav-tags');
            tagsEl.innerHTML = d.detected_signs.length
                ? d.detected_signs.map(s => `id=${s.tag_id} (${s.sign_type || 'unknown'})  area=${s.area}`).join('<br>')
                : 'none';

            const logEl = document.getElementById('nav-log');
            logEl.innerHTML = d.event_log.map(l => l.replace(/&/g,'&amp;').replace(/</g,'&lt;')).join('<br>');
        }).catch(() => {});
    }
    setInterval(refreshNavStatus, 1000);
    refreshNavStatus();

    function setRunningUI(isRunning) {
        const indicator = document.getElementById('run-indicator');
        const label     = document.getElementById('run-label');
        indicator.style.background = isRunning ? '#2ecc71' : '#e74c3c';
        label.textContent = isRunning ? 'RUNNING' : 'STOPPED — camera learning lane width';
        label.style.color = isRunning ? '#2ecc71' : 'var(--text-secondary)';
    }

    function driveStart() {
        postJSON('/start', {}).then(() => setRunningUI(true))
            .catch(() => showStatus('config-status', 'Start failed!', 'error'));
    }

    function driveStop() {
        postJSON('/stop', {}).then(() => setRunningUI(false))
            .catch(() => showStatus('config-status', 'Stop failed!', 'error'));
    }

    // Sync indicator with server state on page load
    fetch('/running').then(r => r.json()).then(d => setRunningUI(d.running));

    // Load HSV bounds from server on page load
    fetch('/get_hsv')
        .then(r => r.json())
        .then(d => {
            setSliderValue('yLowH',  d.yellow_lower_h);
            setSliderValue('yHighH', d.yellow_upper_h);
            setSliderValue('yLowS',  d.yellow_lower_s);
            setSliderValue('yHighS', d.yellow_upper_s);
            setSliderValue('yLowV',  d.yellow_lower_v);
            setSliderValue('yHighV', d.yellow_upper_v);
            setSliderValue('wLowH',  d.white_lower_h);
            setSliderValue('wHighH', d.white_upper_h);
            setSliderValue('wLowS',  d.white_lower_s);
            setSliderValue('wHighS', d.white_upper_s);
            setSliderValue('wLowV',  d.white_lower_v);
            setSliderValue('wHighV', d.white_upper_v);
        });

    const hsvKeys = {
        'yLowH':  'yellow_lower_h', 'yHighH': 'yellow_upper_h',
        'yLowS':  'yellow_lower_s', 'yHighS': 'yellow_upper_s',
        'yLowV':  'yellow_lower_v', 'yHighV': 'yellow_upper_v',
        'wLowH':  'white_lower_h',  'wHighH': 'white_upper_h',
        'wLowS':  'white_lower_s',  'wHighS': 'white_upper_s',
        'wLowV':  'white_lower_v',  'wHighV': 'white_upper_v',
    };

    Object.entries(hsvKeys).forEach(([sliderId, key]) => {
        syncSliderInput(sliderId, () => {
            const payload = {};
            payload[key] = parseInt(document.getElementById(sliderId).value);
            postJSON('/update_hsv', payload)
                .then(() => showStatus('hsv-status', 'HSV Updated!', 'success'));
        });
    });

    document.getElementById('k_d').oninput = function() {
        document.getElementById('k_d_value').textContent = this.value;
    };
    document.getElementById('k_phi').oninput = function() {
        document.getElementById('k_phi_value').textContent = this.value;
    };
    document.getElementById('const').oninput = function() {
        document.getElementById('const_value').textContent = this.value;
    };

    function resetPosition() {
        postJSON('/reset', {})
            .then(() => showStatus('config-status', 'Position Reset!', 'success'))
            .catch(() => showStatus('config-status', 'Reset Failed!', 'error'));
    }

    function updateConfig() {
        postJSON('/update_config', {
            k_d:   parseFloat(document.getElementById('k_d').value),
            k_phi: parseFloat(document.getElementById('k_phi').value),
            const: parseFloat(document.getElementById('const').value)
        })
        .then(() => showStatus('config-status', 'Config Updated!', 'success'))
        .catch(() => showStatus('config-status', 'Update Failed!', 'error'));
    }
'''

APRILTAG_NAVIGATION_TEMPLATE = render_template(
    'AprilTag Navigation — {{ hostname }}',
    '{{ hostname }} — Lane Following + Traffic Sign Reactions',
    _CONTENT,
    extra_css=_EXTRA_CSS,
    extra_js=_JS,
)
