/**
 * KeyboardControlUI - Browser keyboard control for cartesian impedance
 *
 * Captures keydown/keyup on the document and forwards them to the backend
 * via WebSocket. The backend processes key states at 250Hz, identical to pygame.
 *
 * Continuous keys: W/S, A/D, Q/E, I/K, J/L, U/O, Z/X
 * One-shot commands: R (home), M (zero FT), Space (print pose)
 */

// Continuous keys that send key_down/key_up
const CONTINUOUS_KEYS = new Set([
    'w', 's', 'a', 'd', 'q', 'e',
    'i', 'k', 'j', 'l', 'u', 'o',
    'z', 'x'
]);

// One-shot command mapping
const COMMAND_KEYS = {
    'r': 'home',
    'm': 'zero_ft',
    ' ': 'print_pose'
};

export class KeyboardControlUI {
    constructor(robotConnection) {
        this.robotConnection = robotConnection;
        this.enabled = false;
        this.activeKeys = new Set();

        // Bound handlers (for removal)
        this._onKeyDown = this._handleKeyDown.bind(this);
        this._onKeyUp = this._handleKeyUp.bind(this);
        this._onBlur = this._handleBlur.bind(this);
    }

    init() {
        document.addEventListener('keydown', this._onKeyDown);
        document.addEventListener('keyup', this._onKeyUp);
        window.addEventListener('blur', this._onBlur);

        this.setupHelpPanel();
        this.updateStatusDisplay();
    }

    setEnabled(enabled) {
        this.enabled = enabled;
        if (!enabled) {
            this._releaseAllKeys();
        }
        this.updateStatusDisplay();
    }

    _handleKeyDown(event) {
        // Ignore when typing in input fields
        const tag = event.target.tagName;
        if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
        if (!this.enabled) return;

        const key = event.key.toLowerCase();

        // Continuous keys
        if (CONTINUOUS_KEYS.has(key)) {
            event.preventDefault();
            if (!this.activeKeys.has(key)) {
                this.activeKeys.add(key);
                this.robotConnection.sendKeyDown(key);
                this.updateActiveKeyDisplay();
            }
            return;
        }

        // One-shot commands (only on first press, not repeat)
        if (!event.repeat && key in COMMAND_KEYS) {
            event.preventDefault();
            this.robotConnection.sendCommand(COMMAND_KEYS[key]);
            this.flashCommandKey(key);
        }
    }

    _handleKeyUp(event) {
        const key = event.key.toLowerCase();

        if (CONTINUOUS_KEYS.has(key) && this.activeKeys.has(key)) {
            this.activeKeys.delete(key);
            this.robotConnection.sendKeyUp(key);
            this.updateActiveKeyDisplay();
        }
    }

    _handleBlur() {
        // Window lost focus - release everything to stop robot motion
        this._releaseAllKeys();
    }

    _releaseAllKeys() {
        for (const key of this.activeKeys) {
            this.robotConnection.sendKeyUp(key);
        }
        this.activeKeys.clear();
        this.updateActiveKeyDisplay();
    }

    // ========== UI ==========

    setupHelpPanel() {
        const toggleBtn = document.getElementById('toggle-keyboard-panel');
        const panel = document.getElementById('floating-keyboard-panel');
        if (!toggleBtn || !panel) return;

        toggleBtn.addEventListener('click', () => {
            const isVisible = panel.style.display !== 'none';
            panel.style.display = isVisible ? 'none' : 'block';
            toggleBtn.classList.toggle('active', !isVisible);
        });

        const closeBtn = panel.querySelector('.panel-close-btn');
        if (closeBtn) {
            closeBtn.addEventListener('click', () => {
                panel.style.display = 'none';
                toggleBtn.classList.remove('active');
            });
        }
    }

    updateActiveKeyDisplay() {
        const keyElements = document.querySelectorAll('.kb-key[data-key]');
        keyElements.forEach(el => {
            const key = el.getAttribute('data-key');
            el.classList.toggle('active', this.activeKeys.has(key));
        });
    }

    flashCommandKey(key) {
        const el = document.querySelector(`.kb-key[data-key="${key}"]`);
        if (!el) return;

        el.classList.add('flash');
        setTimeout(() => el.classList.remove('flash'), 200);
    }

    updateStatusDisplay() {
        const statusDot = document.querySelector('.kb-status-dot');
        const statusText = document.querySelector('.kb-status-text');

        if (statusDot) {
            statusDot.className = `kb-status-dot ${this.enabled ? 'active' : 'inactive'}`;
        }
        if (statusText) {
            statusText.textContent = this.enabled ? 'Keyboard active' : 'Not connected';
        }
    }
}
