/**
 * ConnectionUI - Robot connection management UI
 * Simplified for cartesian impedance control (read-only visualization)
 */

const CONNECTION_PANEL_OFFSET_X = 50;

export class ConnectionUI {
    constructor(robotConnection, panelManager = null) {
        this.robotConnection = robotConnection;
        this.panelManager = panelManager;
        this.onConnect = null;      // (config) => void
        this.onDisconnect = null;   // () => void
        this.onModeChanged = null;  // (mode) => void

        this.panel = null;
        this.statusIndicator = null;
        this.connectBtn = null;
        this.serverInput = null;

        // Default server URL
        this.serverUrl = 'http://localhost:5001';

        // Control mode
        this.currentMode = 'position';

        // Waypoint state
        this.waypoints = [];
        this.maxWaypoints = 6;
        this.trajectoryRunning = false;
    }

    /**
     * Create and inject connection UI elements
     */
    createUI() {
        this.createConnectionPanel();
        this.createTopBarIndicator();
        this.createControlSections();
        this.setupCallbacks();
    }

    /**
     * Create floating connection panel
     */
    createConnectionPanel() {
        let panelContainer = document.getElementById('floating-connection-panel');
        if (panelContainer) {
            this.panel = panelContainer;
            this.setupPanelElements();
            this.panelManager?.registerPanel('floating-connection-panel');
            return;
        }

        this.panel = document.createElement('div');
        this.panel.id = 'floating-connection-panel';
        this.panel.className = 'floating-panel';
        this.panel.innerHTML = `
            <div class="floating-panel-header">
                    <span data-i18n="robotConnection">机器人连接</span>
                <button class="panel-close-btn" title="Close">×</button>
            </div>
            <div class="floating-panel-content">
                <div class="connection-status-section">
                    <div class="connection-status">
                        <span class="status-dot disconnected"></span>
                        <span class="status-text" data-i18n="disconnected">未连接</span>
                    </div>
                    <div class="connection-mode"></div>
                </div>

                <div class="connection-form">
                    <label for="server-url" data-i18n="serverUrl">服务地址</label>
                    <input type="text" id="server-url" value="http://localhost:5001" placeholder="http://localhost:5001">
                </div>

                <div class="connection-buttons">
                    <button id="connect-btn" class="control-button primary" data-i18n="connect">连接</button>
                    <button id="disconnect-btn" class="control-button" disabled data-i18n="disconnect">断开连接</button>
                </div>

                <div class="robot-info" style="display: none;">
                    <div class="info-row">
                        <span class="info-label">Robot:</span>
                        <span class="info-value" id="robot-name">-</span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">Joints:</span>
                        <span class="info-value" id="joint-count">-</span>
                    </div>
                    <div class="info-row">
                        <span class="info-label">Mode:</span>
                        <span class="info-value" id="connection-mode">-</span>
                    </div>
                </div>
            </div>
        `;

        document.body.appendChild(this.panel);
        this.setupPanelElements();
        this.panelManager?.registerPanel('floating-connection-panel');
    }

    /**
     * Setup panel element references and events
     */
    setupPanelElements() {
        this.serverInput = this.panel.querySelector('#server-url');
        this.connectBtn = this.panel.querySelector('#connect-btn');
        const disconnectBtn = this.panel.querySelector('#disconnect-btn');
        const closeBtn = this.panel.querySelector('.panel-close-btn');

        if (this.connectBtn) {
            this.connectBtn.addEventListener('click', () => this.handleConnect());
        }

        if (disconnectBtn) {
            disconnectBtn.addEventListener('click', () => this.handleDisconnect());
        }

        if (closeBtn) {
            closeBtn.addEventListener('click', () => this.hidePanel());
        }
    }

    /**
     * Create top bar connection indicator
     */
    createTopBarIndicator() {
        const topBar = document.getElementById('top-control-bar');
        if (!topBar) return;

        if (document.getElementById('connection-indicator')) return;

        const divider = document.createElement('div');
        divider.className = 'control-bar-divider';

        this.statusIndicator = document.createElement('button');
        this.statusIndicator.id = 'connection-indicator';
        this.statusIndicator.className = 'tool-button';
        this.statusIndicator.innerHTML = `
            <span class="status-dot-small disconnected"></span>
            <span class="connection-label" data-i18n="robotConnection">机器人连接</span>
        `;
        this.statusIndicator.title = window.i18n?.t('robotConnection') || 'Robot Connection';
        this.statusIndicator.addEventListener('click', () => this.togglePanel());
        this.statusIndicator.classList.toggle('active', this.panel ? !this.panel.classList.contains('hidden') : false);

        topBar.insertBefore(this.statusIndicator, topBar.firstChild);
        topBar.insertBefore(divider, this.statusIndicator.nextSibling);

        requestAnimationFrame(() => this.positionInitialPanel());
    }

    positionInitialPanel() {
        if (!this.panel || !this.statusIndicator || !this.panelManager) return;
        if (!this.panelManager.isPanelVisible('floating-connection-panel')) return;

        this.panelManager.positionPanelBelowElement(
            'floating-connection-panel',
            this.statusIndicator,
            'center',
            10,
            CONNECTION_PANEL_OFFSET_X
        );
        this.statusIndicator.classList.add('active');
    }

    /**
     * Create control sections (mode buttons, quick controls, waypoints)
     * Appended to the connection panel after robot-info
     */
    createControlSections() {
        if (!this.panel) return;
        const content = this.panel.querySelector('.floating-panel-content');
        if (!content) return;
        if (document.getElementById('control-sections')) return;

        const sections = document.createElement('div');
        sections.id = 'control-sections';
        sections.style.display = 'none';

        sections.innerHTML = `
            <!-- Control Mode -->
            <div class="robot-controls" style="margin-top: 12px;">
                <div style="font-size:11px;font-weight:600;color:var(--text-secondary);margin-bottom:8px;text-transform:uppercase;letter-spacing:0.5px;" data-i18n="controlMode">控制模式</div>
                <div id="mode-buttons" style="display:flex;gap:4px;">
                    <button class="mode-btn control-button active" data-mode="position" style="flex:1;padding:6px 8px;font-size:11px;border-radius:6px;" data-i18n="positionMode">位置控制</button>
                    <button class="mode-btn control-button" data-mode="gravity_comp" style="flex:1;padding:6px 8px;font-size:11px;border-radius:6px;" data-i18n="gravityMode">重力补偿</button>
                    <button class="mode-btn control-button" data-mode="gravity_friction" style="flex:1;padding:6px 8px;font-size:11px;border-radius:6px;" data-i18n="gravityFrictionMode">重力+摩擦补偿</button>
                    <button class="mode-btn control-button" data-mode="impedance" style="flex:1;padding:6px 8px;font-size:11px;border-radius:6px;" data-i18n="impedanceMode">阻抗控制</button>
                </div>
            </div>

            <!-- Waypoints -->
            <div id="waypoints-section" class="robot-controls" style="margin-top: 8px;">
                <div class="waypoints-content">
                    <div style="font-size:11px;font-weight:600;color:var(--text-secondary);margin-bottom:8px;text-transform:uppercase;letter-spacing:0.5px;">
                        <span data-i18n="waypoints">轨迹点</span> <span id="wp-count" style="color:var(--text-tertiary);font-weight:400;">(0/6)</span>
                    </div>
                    <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-bottom:6px;">
                        <button id="wp-add" class="control-button" style="padding:8px;font-size:12px;" data-i18n="addCurrent">+ 添加当前位置</button>
                        <button id="wp-clear" class="control-button danger" style="padding:8px;font-size:12px;" data-i18n="clear">清空</button>
                    </div>
                    <div id="wp-list" style="max-height:120px;overflow-y:auto;margin-bottom:6px;font-size:11px;color:var(--text-tertiary);">
                        <div style="text-align:center;padding:8px;">No waypoints</div>
                    </div>
                    <div style="margin-bottom:6px;">
                        <button id="wp-go-first" class="control-button" style="width:100%;padding:8px;font-size:12px;">Move to Point 1</button>
                    </div>
                    <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;">
                        <button id="wp-run" class="control-button" style="padding:8px;font-size:12px;" data-i18n="runTrajectory">▶ 执行轨迹</button>
                        <button id="wp-stop" class="control-button danger" style="padding:8px;font-size:12px;" disabled data-i18n="stop">■ 停止</button>
                    </div>
                </div>
                <div class="waypoints-lock-overlay">Disabled in impedance</div>
            </div>

            <!-- Reusable graphical tasks -->
            <div id="tasks-section" class="robot-controls" style="margin-top: 12px;">
                <div style="font-size:11px;font-weight:600;color:var(--text-secondary);margin-bottom:8px;text-transform:uppercase;letter-spacing:0.5px;">抓取任务</div>
                <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-bottom:6px;">
                    <button id="demo-run" class="control-button primary" style="padding:8px;font-size:12px;">▶ 演示</button>
                    <button id="demo-stop" class="control-button danger" style="padding:8px;font-size:12px;" disabled>■ 停止</button>
                </div>
                <div id="demo-status" style="font-size:11px;color:var(--text-tertiary);margin-bottom:8px;">演示 = 回放学习到的抓取轨迹（约80秒）</div>
                <div style="font-size:11px;font-weight:600;color:var(--text-secondary);margin:8px 0 6px;text-transform:uppercase;letter-spacing:0.5px;">视觉测试（V1）</div>
                <div style="display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-bottom:6px;">
                    <button id="vision-check" class="control-button" style="padding:8px;font-size:12px;">👁 视觉检测</button>
                    <button id="vision-grasp" class="control-button primary" style="padding:8px;font-size:12px;">🦾 视觉抓取</button>
                </div>
                <div style="display:flex;gap:6px;margin-bottom:6px;">
                    <input id="vision-label" type="text" placeholder="采样标签（如 p0 / x+5cm）" style="flex:1;min-width:0;padding:7px 8px;border-radius:6px;border:1px solid var(--glass-border);background:var(--glass-bg);color:var(--text-primary);font-size:12px;">
                    <button id="vision-probe" class="control-button" style="padding:7px 9px;font-size:12px;">📐 采样</button>
                </div>
                <div id="vision-status" style="font-size:11px;color:var(--text-tertiary);margin-bottom:8px;">检测 = 只判定不动臂；抓取 = 判定通过后自动回放；采样 = 记录检测中心（V2 标定用）</div>
                <div style="display:flex;gap:6px;margin-bottom:6px;">
                    <button id="vision-samples-toggle" class="control-button" style="flex:1;padding:7px 6px;font-size:12px;">📋 采样数据</button>
                    <button id="vision-samples-rebuild" class="control-button" style="flex:1;padding:7px 6px;font-size:12px;">🔄 重建映射</button>
                </div>
                <div id="vision-samples-area" style="display:none;border:1px solid var(--glass-border);border-radius:6px;padding:6px;margin-bottom:8px;"></div>
                <div style="display:flex;gap:6px;margin-bottom:6px;">
                    <input id="task-name" type="text" placeholder="任务名称" value="固定位置抓取" style="flex:1;min-width:0;padding:7px 8px;border-radius:6px;border:1px solid var(--glass-border);background:var(--glass-bg);color:var(--text-primary);font-size:12px;">
                    <button id="task-save" class="control-button primary" style="padding:7px 9px;font-size:12px;">保存</button>
                </div>
                <div style="display:flex;gap:6px;margin-bottom:6px;">
                    <select id="task-list" style="flex:1;min-width:0;padding:7px 8px;border-radius:6px;border:1px solid var(--glass-border);background:var(--glass-bg);color:var(--text-primary);font-size:12px;"><option value="">暂无任务</option></select>
                    <button id="task-refresh" class="control-button" style="padding:7px 9px;font-size:12px;">刷新</button>
                </div>
                <div style="display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;">
                    <button id="task-load" class="control-button" style="padding:7px 6px;font-size:12px;">加载</button>
                    <button id="task-run" class="control-button" style="padding:7px 6px;font-size:12px;">执行</button>
                    <button id="task-delete" class="control-button danger" style="padding:7px 6px;font-size:12px;">删除</button>
                </div>
                <button id="task-stop" class="control-button danger" style="width:100%;padding:7px;font-size:12px;margin-top:6px;" disabled>停止任务</button>
                <div id="task-status" style="font-size:11px;color:var(--text-tertiary);margin-top:6px;">先添加至少两个轨迹点，再保存任务</div>
            </div>
        `;

        content.appendChild(sections);
        this.bindControlEvents();
        this.updateWaypointLockState();
        this.updateWaypointButtonState();
    }

    /** Bind control section button events */
    bindControlEvents() {
        // Mode buttons
        document.querySelectorAll('.mode-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const mode = btn.getAttribute('data-mode');
                this.setControlMode(mode);
            });
        });

        // Waypoints
        document.getElementById('wp-add')?.addEventListener('click', () => this.addWaypoint());
        document.getElementById('wp-clear')?.addEventListener('click', () => this.clearWaypoints());
        document.getElementById('wp-go-first')?.addEventListener('click', () => this.goToFirstWaypoint());
        document.getElementById('wp-run')?.addEventListener('click', () => this.runTrajectory());
        document.getElementById('wp-stop')?.addEventListener('click', () => this.stopTrajectory());
        document.getElementById('task-save')?.addEventListener('click', () => this.saveTask());
        document.getElementById('task-refresh')?.addEventListener('click', () => this.refreshTasks());
        document.getElementById('task-load')?.addEventListener('click', () => this.loadTask());
        document.getElementById('task-run')?.addEventListener('click', () => this.runTask());
        document.getElementById('task-delete')?.addEventListener('click', () => this.deleteTask());
        document.getElementById('task-stop')?.addEventListener('click', () => this.stopTask());

        // 演示：回放精炼抓取轨迹
        document.getElementById('demo-run')?.addEventListener('click', () => this.startDemo());
        document.getElementById('demo-stop')?.addEventListener('click', () => this.stopDemo());
        // 视觉便捷测试：检测 / 抓取 / 采样
        document.getElementById('vision-check')?.addEventListener('click', () => this.runVision({ action: 'check', live: false }));
        document.getElementById('vision-grasp')?.addEventListener('click', () => this.runVision({ action: 'check', live: true, speed: 1.0 }));
        document.getElementById('vision-probe')?.addEventListener('click', () => {
            const label = (document.getElementById('vision-label')?.value || '').trim();
            if (!label) {
                document.getElementById('vision-status').textContent = '请先填写采样标签';
                return;
            }
            this.runVision({ action: 'probe', label });
        });
        // 采样数据表格：展开/收起、重建像素映射、行内保存/删除（事件委托）
        document.getElementById('vision-samples-toggle')?.addEventListener('click', () => this.toggleVisionSamples());
        document.getElementById('vision-samples-rebuild')?.addEventListener('click', () => this.rebuildPixelMap());
        document.getElementById('vision-samples-area')?.addEventListener('click', (e) => {
            const save = e.target.closest('[data-vs-save]');
            const del = e.target.closest('[data-vs-del]');
            if (save) this.saveVisionSample(parseInt(save.dataset.vsSave, 10));
            else if (del) this.deleteVisionSample(parseInt(del.dataset.vsDel, 10));
        });
    }

    /** 视觉便捷测试：调 /api/vision/run；live 模式触发回放后复用演示进度轮询 */
    async runVision({ action, live = false, speed = 1.0, label = '' } = {}) {
        const statusEl = document.getElementById('vision-status');
        const checkBtn = document.getElementById('vision-check');
        const graspBtn = document.getElementById('vision-grasp');
        const probeBtn = document.getElementById('vision-probe');
        if (!statusEl) return;
        if (live && !this.robotConnection?.isConnected?.()) {
            statusEl.textContent = '请先连接机械臂';
            return;
        }
        [checkBtn, graspBtn, probeBtn].forEach(b => { if (b) b.disabled = true; });
        statusEl.textContent = action === 'probe' ? '● 采样中（约 10 秒）...' : '● 视觉判定中...';
        try {
            const res = await fetch('/api/vision/run', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ action, live, speed, label })
            });
            const data = await res.json();
            const line = (data.output || '').split('\n').filter(l => l.includes('->') || l.includes('[probe]')).pop() || data.output || '';
            if (live && data.replay?.success) {
                statusEl.textContent = `✅ 判定通过，已触发回放（速度 x${speed}）`;
                if (graspBtn) graspBtn.disabled = false;
                document.getElementById('demo-run').disabled = true;
                document.getElementById('demo-stop').disabled = false;
                this._demoTimer = setInterval(() => this.pollDemoStatus(), 1000);
                return;
            }
            statusEl.textContent = data.replay && !data.replay.success
                ? `⚠ 判定通过但回放未启动: ${data.replay.error || ''}`
                : (data.success ? `✅ ${line}` : `⚠ ${line || data.error || '执行失败'}`);
        } catch (e) {
            statusEl.textContent = `请求失败: ${e.message}`;
        } finally {
            [checkBtn, graspBtn, probeBtn].forEach(b => { if (b) b.disabled = false; });
        }
    }

    /** 展开/收起采样数据表格 */
    toggleVisionSamples() {
        const area = document.getElementById('vision-samples-area');
        if (!area) return;
        if (area.style.display === 'none') {
            area.style.display = 'block';
            this.loadVisionSamples();
        } else {
            area.style.display = 'none';
        }
    }

    /** 拉取采样记录 + 像素映射状态，渲染可编辑表格 */
    async loadVisionSamples() {
        const area = document.getElementById('vision-samples-area');
        if (!area) return;
        try {
            const res = await fetch('/api/vision/samples');
            const data = await res.json();
            area.innerHTML = this._renderSamplesTable(data);
        } catch (e) {
            area.innerHTML = `<div style="font-size:11px;color:var(--text-tertiary);">加载失败: ${e.message}</div>`;
        }
    }

    _renderSamplesTable({ samples = [], pixel_map }) {
        const inp = 'padding:3px 4px;border-radius:4px;border:1px solid var(--glass-border);background:var(--glass-bg);color:var(--text-primary);font-size:11px;';
        let html = '<div style="font-size:10px;color:var(--text-tertiary);margin-bottom:4px;line-height:1.5;">';
        if (pixel_map) {
            const sx = Math.hypot(pixel_map.u_x[0], pixel_map.u_x[1]) / 5;
            const sy = Math.hypot(pixel_map.u_y[0], pixel_map.u_y[1]) / 5;
            html += `当前映射：p0=(${pixel_map.p0.map(v => Math.round(v)).join(',')}) · `
                + `前方(X) ${sx.toFixed(0)}px/cm · 左方(Y) ${sy.toFixed(0)}px/cm`
                + `${pixel_map.built ? ' · ' + pixel_map.built : ''}`;
        } else {
            html += '映射未生成：需 p0 / x+5 / y+5 三点；编辑或删除数据后需点“重建映射”';
        }
        html += '</div>';
        if (!samples.length) {
            html += '<div style="font-size:11px;color:var(--text-tertiary);">暂无采样数据（用 📐采样 记录）</div>';
            return html;
        }
        html += '<table style="width:100%;border-collapse:collapse;font-size:11px;">'
            + '<tr style="color:var(--text-secondary);">'
            + '<th style="text-align:left;padding:2px 3px;">标签</th>'
            + '<th style="text-align:right;padding:2px 3px;">中心x</th>'
            + '<th style="text-align:right;padding:2px 3px;">中心y</th>'
            + '<th style="text-align:right;padding:2px 3px;">σ</th>'
            + '<th style="text-align:right;padding:2px 3px;">n</th>'
            + '<th style="text-align:left;padding:2px 3px;">时间</th><th></th></tr>';
        samples.forEach((rec, idx) => {
            const ed = rec.edited ? '<span title="已人工修改" style="color:var(--text-tertiary);">✎</span>' : '';
            const sigma = rec.std ? `${rec.std[0].toFixed(0)},${rec.std[1].toFixed(0)}` : '-';
            html += `<tr style="border-top:1px solid var(--glass-border);">
                <td style="padding:3px 2px;"><input id="vs-label-${idx}" value="${String(rec.label || '').replace(/"/g, '&quot;')}" style="width:50px;${inp}">${ed}</td>
                <td><input id="vs-x-${idx}" type="number" step="0.1" value="${rec.mean?.[0] ?? ''}" style="width:54px;${inp}"></td>
                <td><input id="vs-y-${idx}" type="number" step="0.1" value="${rec.mean?.[1] ?? ''}" style="width:54px;${inp}"></td>
                <td style="text-align:right;color:var(--text-tertiary);">${sigma}</td>
                <td style="text-align:right;color:var(--text-tertiary);">${rec.n ?? '-'}</td>
                <td style="color:var(--text-tertiary);font-size:10px;">${String(rec.time || '').slice(5, 16)}</td>
                <td style="white-space:nowrap;padding-left:3px;">
                    <button class="control-button" data-vs-save="${idx}" style="padding:3px 6px;font-size:10px;">存</button>
                    <button class="control-button danger" data-vs-del="${idx}" style="padding:3px 6px;font-size:10px;">删</button>
                </td></tr>`;
        });
        html += '</table>';
        return html;
    }

    /** 保存单行编辑（标签/中心坐标） */
    async saveVisionSample(idx) {
        const statusEl = document.getElementById('vision-status');
        const label = document.getElementById(`vs-label-${idx}`)?.value.trim();
        const x = parseFloat(document.getElementById(`vs-x-${idx}`)?.value);
        const y = parseFloat(document.getElementById(`vs-y-${idx}`)?.value);
        if (!label || Number.isNaN(x) || Number.isNaN(y)) {
            statusEl.textContent = '⚠ 数据不完整（标签/中心x/中心y 都必填）';
            return;
        }
        try {
            const res = await fetch(`/api/vision/samples/${idx}`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ label, mean: [x, y] })
            });
            const data = await res.json();
            statusEl.textContent = data.success ? `✅ ${data.note || '已保存'}` : `⚠ 保存失败: ${data.error || ''}`;
            if (data.success) this.loadVisionSamples();
        } catch (e) {
            statusEl.textContent = `请求失败: ${e.message}`;
        }
    }

    /** 删除单行采样（确认后） */
    async deleteVisionSample(idx) {
        if (!window.confirm(`确定删除第 ${idx + 1} 条采样数据？`)) return;
        const statusEl = document.getElementById('vision-status');
        try {
            const res = await fetch(`/api/vision/samples/${idx}`, { method: 'DELETE' });
            const data = await res.json();
            statusEl.textContent = data.success ? `✅ ${data.note || '已删除'}` : `⚠ 删除失败: ${data.error || ''}`;
            if (data.success) this.loadVisionSamples();
        } catch (e) {
            statusEl.textContent = `请求失败: ${e.message}`;
        }
    }

    /** 重建像素映射（编辑/删除采样后调用，映射自动失效保护） */
    async rebuildPixelMap() {
        const btn = document.getElementById('vision-samples-rebuild');
        const statusEl = document.getElementById('vision-status');
        if (btn) btn.disabled = true;
        statusEl.textContent = '● 重建像素映射中...';
        try {
            const res = await fetch('/api/vision/samples/calmap', { method: 'POST' });
            const data = await res.json();
            statusEl.textContent = `${data.success ? '✅' : '⚠'} ${data.message || ''}`;
            this.loadVisionSamples();
        } catch (e) {
            statusEl.textContent = `请求失败: ${e.message}`;
        } finally {
            if (btn) btn.disabled = false;
        }
    }

    /** 演示：启动精炼轨迹回放 */
    async startDemo() {
        const btn = document.getElementById('demo-run');
        const statusEl = document.getElementById('demo-status');
        if (!btn || !statusEl) return;
        if (!this.robotConnection?.isConnected?.()) {
            statusEl.textContent = '请先连接机械臂';
            return;
        }
        btn.disabled = true;
        statusEl.textContent = '演示启动中...';
        try {
            const res = await fetch('/api/replay/start', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ speed: 1.0 })
            });
            const data = await res.json();
            if (!data.success) {
                statusEl.textContent = `启动失败: ${data.error || '未知错误'}`;
                btn.disabled = false;
                return;
            }
            document.getElementById('demo-stop').disabled = false;
            this._demoTimer = setInterval(() => this.pollDemoStatus(), 1000);
            statusEl.textContent = '● 演示中...';
        } catch (e) {
            statusEl.textContent = `启动失败: ${e.message}`;
            btn.disabled = false;
        }
    }

    /** 停止演示 */
    async stopDemo() {
        try { await fetch('/api/replay/stop', { method: 'POST' }); } catch (e) { /* ignore */ }
    }

    /** 轮询演示进度 */
    async pollDemoStatus() {
        const btn = document.getElementById('demo-run');
        const stopBtn = document.getElementById('demo-stop');
        const statusEl = document.getElementById('demo-status');
        if (!statusEl) return;
        try {
            const res = await fetch('/api/replay/status');
            const s = await res.json();
            if (!s.running) {
                clearInterval(this._demoTimer);
                btn.disabled = false;
                stopBtn.disabled = true;
                statusEl.textContent = s.stage === 'done' ? '✅ 演示完成' : `已停止 (${s.stage})`;
                return;
            }
            const pct = Math.round((s.progress || 0) * 100);
            const stageMap = { move_to_start: '对齐起始位', playing: '执行轨迹', finishing: '收尾' };
            const stage = String(s.stage || '').startsWith('gripper_event') ? '夹爪动作' : (stageMap[s.stage] || s.stage);
            statusEl.textContent = `● 演示中 ${pct}% · ${stage} · 夹爪事件 ${s.events_done}/4`;
        } catch (e) { /* 下次轮询重试 */ }
    }

    /** Set control mode via WebSocket */
    setControlMode(mode) {
        if (!this.robotConnection.isConnected()) return;
        this.robotConnection.setMode(mode);
        this.currentMode = mode;
        document.querySelectorAll('.mode-btn').forEach(b => {
            b.classList.toggle('active', b.getAttribute('data-mode') === mode);
        });
        this.updateWaypointLockState();
        this.updateWaypointButtonState();
        if (this.onModeChanged) this.onModeChanged(mode);
    }

    isWaypointLocked() {
        return this.currentMode === 'impedance';
    }

    updateWaypointLockState() {
        const section = document.getElementById('waypoints-section');
        if (section) section.classList.toggle('waypoints-locked', this.isWaypointLocked());
    }

    updateWaypointButtonState() {
        const connected = Boolean(this.robotConnection?.isConnected?.());
        const locked = this.isWaypointLocked();
        const addBtn = document.getElementById('wp-add');
        const clearBtn = document.getElementById('wp-clear');
        const goFirstBtn = document.getElementById('wp-go-first');
        const runBtn = document.getElementById('wp-run');
        const stopBtn = document.getElementById('wp-stop');

        if (addBtn) addBtn.disabled = locked || !connected || this.waypoints.length >= this.maxWaypoints;
        if (clearBtn) clearBtn.disabled = locked || !connected || this.waypoints.length === 0;
        if (goFirstBtn) goFirstBtn.disabled = locked || !connected || this.trajectoryRunning || this.waypoints.length < 1;
        if (runBtn) runBtn.disabled = locked || !connected || this.trajectoryRunning || this.waypoints.length < 2;
        if (stopBtn) stopBtn.disabled = locked || !connected || !this.trajectoryRunning;
        const taskButtons = ['task-save', 'task-load', 'task-run', 'task-delete', 'task-refresh'];
        taskButtons.forEach(id => { const btn = document.getElementById(id); if (btn) btn.disabled = locked || !connected; });
        const taskStop = document.getElementById('task-stop');
        if (taskStop) taskStop.disabled = locked || !connected || !this.trajectoryRunning;
    }

    /** Add current robot position as waypoint */
    addWaypoint() {
        if (this.isWaypointLocked()) return;
        if (!this.robotConnection.isConnected()) return;
        const state = this.robotConnection.getState();
        if (!state || !state.positions || state.positions.length === 0) return;

        if (this.waypoints.length >= this.maxWaypoints) return;

        const waypoint = {
            positions: [...state.positions],
            gripper: Number.isFinite(Number(state.gripper_position)) ? Number(state.gripper_position) : 0.0,
            duration: 1.0,
            settle: 0.8,
            index: this.waypoints.length
        };
        this.waypoints.push(waypoint);
        this.robotConnection.socket.emit('add_waypoint', {
            positions: waypoint.positions,
            gripper: waypoint.gripper,
            duration: waypoint.duration
            ,settle: waypoint.settle
        });
        this.updateWaypointUI();
    }

    /** Clear all waypoints */
    clearWaypoints() {
        if (this.isWaypointLocked() || !this.robotConnection.isConnected()) return;
        this.waypoints = [];
        this.robotConnection.socket.emit('clear_waypoints');
        this.updateWaypointUI();
    }

    /** Move to the first waypoint */
    goToFirstWaypoint() {
        if (this.isWaypointLocked() || !this.robotConnection.isConnected()) return;
        if (this.trajectoryRunning || this.waypoints.length < 1) return;
        this.robotConnection.socket.emit('go_to_waypoint', { index: 0 });
    }

    /** Run trajectory */
    runTrajectory() {
        if (this.isWaypointLocked() || !this.robotConnection.isConnected()) return;
        if (this.waypoints.length < 2) return;
        if (this.currentMode === 'gravity_comp' || this.currentMode === 'gravity_friction') {
            this.setControlMode('position');
        }
        this.trajectoryRunning = true;
        this.robotConnection.socket.emit('run_trajectory', { control_rate: 100 });
        this.updateWaypointButtonState();
    }

    /** Stop trajectory */
    stopTrajectory() {
        if (this.isWaypointLocked() || !this.robotConnection.isConnected()) return;
        this.trajectoryRunning = false;
        this.robotConnection.socket.emit('stop_trajectory');
        this.updateWaypointButtonState();
    }

    /** Update waypoint list UI */
    updateWaypointUI() {
        const list = document.getElementById('wp-list');
        const count = document.getElementById('wp-count');
        if (!list) return;
        if (count) count.textContent = `(${this.waypoints.length}/${this.maxWaypoints})`;

        if (this.waypoints.length === 0) {
            list.innerHTML = '<div style="text-align:center;padding:8px;">No waypoints</div>';
            this.updateWaypointButtonState();
            return;
        }

        list.innerHTML = this.waypoints.map((wp, i) => {
            const posStr = wp.positions.map(p => p.toFixed(2)).join(', ');
            const gripperStr = Number.isFinite(Number(wp.gripper)) ? ` | 夹爪:${Number(wp.gripper).toFixed(2)}` : '';
            const dur = (wp.duration || 1.0).toFixed(1);
            const settle = (wp.settle ?? 0.8).toFixed(1);
            return `<div style="padding:5px 6px;border-bottom:1px solid rgba(255,255,255,0.06);display:flex;align-items:center;gap:5px;">
                <span style="color:var(--accent);font-weight:500;">#${i+1}</span>
                <span style="color:var(--text-tertiary);flex:1;">[${posStr}]${gripperStr}</span>
                <label title="到位后等待时间" style="color:var(--text-tertiary);white-space:nowrap;">停<input data-wp-settle="${i}" type="number" min="0" max="10" step="0.1" value="${settle}" style="width:42px;margin-left:2px;background:var(--glass-bg);border:1px solid var(--glass-border);color:var(--text-primary);border-radius:3px;padding:2px;font-size:10px;">s</label>
                <label title="到达该点的动作时间" style="color:var(--text-tertiary);white-space:nowrap;">时<input data-wp-duration="${i}" type="number" min="0.1" max="300" step="0.1" value="${dur}" style="width:42px;margin-left:2px;background:var(--glass-bg);border:1px solid var(--glass-border);color:var(--text-primary);border-radius:3px;padding:2px;font-size:10px;">s</label>
            </div>`;
        }).join('');
        list.querySelectorAll('[data-wp-settle]').forEach(input => input.addEventListener('change', () => {
            const index = Number(input.dataset.wpSettle);
            const value = Math.max(0, Math.min(10, Number(input.value) || 0));
            if (!this.waypoints[index]) return;
            this.waypoints[index].settle = value;
            this.robotConnection.socket.emit('update_waypoint_settle', {index, settle: value});
        }));
        list.querySelectorAll('[data-wp-duration]').forEach(input => input.addEventListener('change', () => {
            const index = Number(input.dataset.wpDuration);
            const value = Math.max(0.1, Math.min(300, Number(input.value) || 1));
            if (!this.waypoints[index]) return;
            this.waypoints[index].duration = value;
            this.robotConnection.socket.emit('update_waypoint_duration', {index, duration: value});
        }));
        this.updateWaypointButtonState();
        const status = document.getElementById('task-status');
        if (status && this.waypoints.length >= 2) status.textContent = '当前轨迹可保存为可重复执行的任务';
    }

    async refreshTasks() {
        if (!this.serverUrl) return;
        try {
            const response = await fetch(`${this.serverUrl}/api/tasks`);
            const data = await response.json();
            const select = document.getElementById('task-list');
            if (!select) return;
            select.innerHTML = data.tasks?.length
                ? data.tasks.map(t => `<option value="${this.escapeHtml(t.name)}">${this.escapeHtml(t.name)}（${t.steps}步）</option>`).join('')
                : '<option value="">暂无任务</option>';
            this.updateTaskStatus(data.running ? `正在执行：${data.active || ''}` : '任务列表已更新');
        } catch (e) { this.showError('读取任务列表失败'); }
    }

    escapeHtml(value) {
        return String(value).replace(/[&<>"]/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
    }

    selectedTaskName() { return document.getElementById('task-list')?.value || ''; }
    updateTaskStatus(text) { const el = document.getElementById('task-status'); if (el) el.textContent = text; }

    async saveTask() {
        const name = document.getElementById('task-name')?.value.trim();
        if (!name) return this.showError('请先输入任务名称');
        if (this.waypoints.length < 2) return this.showError('请先添加至少两个轨迹点');
        try {
            const r = await fetch(`${this.serverUrl}/api/tasks`, { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({name}) });
            const data = await r.json();
            if (!r.ok) throw new Error(data.error || '保存失败');
            this.updateTaskStatus(`已保存任务：${data.task.name}`);
            await this.refreshTasks();
        } catch (e) { this.showError(e.message); }
    }

    async loadTask() {
        const name = this.selectedTaskName(); if (!name) return;
        try {
            const r = await fetch(`${this.serverUrl}/api/tasks/${encodeURIComponent(name)}/load`, {method:'POST'});
            const data = await r.json(); if (!r.ok) throw new Error(data.error || '加载失败');
            this.waypoints = data.waypoints || [];
            this.updateWaypointUI(); this.updateTaskStatus(`已加载任务：${name}`);
        } catch (e) { this.showError(e.message); }
    }

    async runTask() {
        const name = this.selectedTaskName(); if (!name) return;
        try {
            const r = await fetch(`${this.serverUrl}/api/tasks/${encodeURIComponent(name)}/run`, {method:'POST'});
            const data = await r.json(); if (!r.ok) throw new Error(data.error || '执行失败');
            this.trajectoryRunning = true; this.updateWaypointButtonState(); this.updateTaskStatus(`正在执行：${name}`);
        } catch (e) { this.showError(e.message); }
    }

    async deleteTask() {
        const name = this.selectedTaskName(); if (!name) return;
        if (!window.confirm(`确定删除任务“${name}”吗？`)) return;
        try {
            const r = await fetch(`${this.serverUrl}/api/tasks/${encodeURIComponent(name)}`, {method:'DELETE'});
            const data = await r.json(); if (!r.ok) throw new Error(data.error || '删除失败');
            await this.refreshTasks(); this.updateTaskStatus(`已删除任务：${name}`);
        } catch (e) { this.showError(e.message); }
    }

    async stopTask() {
        try { await fetch(`${this.serverUrl}/api/tasks/stop`, {method:'POST'}); }
        finally { this.trajectoryRunning = false; this.updateWaypointButtonState(); this.updateTaskStatus('任务已停止'); }
    }

    /**
     * Setup robot connection callbacks
     */
    setupCallbacks() {
        this.robotConnection.onConnected = () => {
            this.updateConnectionStatus(true);
            this.bindSocketEvents();
            this.updateWaypointLockState();
            this.updateWaypointButtonState();
            this.refreshTasks();
        };

        this.robotConnection.onDisconnected = () => {
            this.updateConnectionStatus(false);
            this.waypoints = [];
            this.trajectoryRunning = false;
            this.updateWaypointLockState();
            this.updateWaypointButtonState();
            if (this.onDisconnect) this.onDisconnect();
        };

        this.robotConnection.onConfigReceived = (config) => {
            this.updateRobotInfo(config);
            if (config.control_mode) {
                this.currentMode = config.control_mode;
                document.querySelectorAll('.mode-btn').forEach(b => {
                    b.classList.toggle('active', b.getAttribute('data-mode') === config.control_mode);
                });
                this.updateWaypointLockState();
                this.updateWaypointButtonState();
                if (this.onModeChanged) this.onModeChanged(config.control_mode);
            }
            if (this.onConnect) this.onConnect(config);
        };

        this.robotConnection.onError = (error) => {
            this.showError(error.message || 'Connection failed');
        };

        // Mode changed callback from RobotConnection
        this.robotConnection.onModeChanged = (mode) => {
            this.currentMode = mode;
            document.querySelectorAll('.mode-btn').forEach(b => {
                b.classList.toggle('active', b.getAttribute('data-mode') === mode);
            });
            this.updateWaypointLockState();
            this.updateWaypointButtonState();
            if (this.onModeChanged) this.onModeChanged(mode);
        };
    }

    /** Bind WebSocket events after connection */
    bindSocketEvents() {
        const sock = this.robotConnection.socket;
        if (!sock) return;

        sock.off('waypoints_updated');
        sock.off('trajectory_complete');
        sock.off('trajectory_error');

        sock.on('waypoints_updated', (data) => {
            if (data.waypoints) {
                this.waypoints = data.waypoints;
                this.updateWaypointUI();
            }
        });

        sock.on('trajectory_complete', () => {
            this.trajectoryRunning = false;
            this.updateWaypointButtonState();
        });

        sock.on('trajectory_error', (data) => {
            this.trajectoryRunning = false;
            this.updateWaypointButtonState();
            if (data.error) this.showError(data.error);
        });
    }

    /**
     * Handle connect button click
     */
    async handleConnect() {
        const url = this.serverInput ? this.serverInput.value.trim() : this.serverUrl;
        if (!url) {
            this.showError('Please enter server URL');
            return;
        }

        this.serverUrl = url;
        this.setConnecting(true);

        try {
            await this.robotConnection.connect(url);
        } catch (error) {
            this.showError(error.message || 'Failed to connect');
            this.setConnecting(false);
        }
    }

    /**
     * Handle disconnect button click
     */
    handleDisconnect() {
        this.robotConnection.disconnect();
        this.updateConnectionStatus(false);
    }

    /**
     * Update connection status display
     */
    updateConnectionStatus(connected) {
        const statusDot = this.panel?.querySelector('.status-dot');
        const statusText = this.panel?.querySelector('.status-text');
        const connectBtn = this.panel?.querySelector('#connect-btn');
        const disconnectBtn = this.panel?.querySelector('#disconnect-btn');
        const robotInfo = this.panel?.querySelector('.robot-info');
        const controlSections = document.getElementById('control-sections');

        if (statusDot) {
            statusDot.className = `status-dot ${connected ? 'connected' : 'disconnected'}`;
        }
        if (statusText) {
            statusText.textContent = window.i18n?.t(connected ? 'connected' : 'disconnected') || (connected ? 'Connected' : 'Disconnected');
        }
        if (connectBtn) {
            connectBtn.disabled = connected;
            connectBtn.textContent = window.i18n?.t('connect') || 'Connect';
        }
        if (disconnectBtn) {
            disconnectBtn.disabled = !connected;
        }
        if (robotInfo) {
            robotInfo.style.display = connected ? 'block' : 'none';
        }
        if (controlSections) {
            controlSections.style.display = connected ? 'block' : 'none';
        }

        // Update top bar indicator
        if (this.statusIndicator) {
            const dot = this.statusIndicator.querySelector('.status-dot-small');
            const label = this.statusIndicator.querySelector('.connection-label');

            if (dot) {
                dot.className = `status-dot-small ${connected ? 'connected' : 'disconnected'}`;
            }
            if (label) {
                label.textContent = window.i18n?.t('robotConnection') || 'Robot Connection';
            }

            const panelOpen = this.panelManager?.isPanelVisible('floating-connection-panel') ||
                (this.panel && !this.panel.classList.contains('hidden'));
            this.statusIndicator.classList.toggle('active', Boolean(panelOpen));
        }
    }

    /**
     * Update robot info display
     */
    updateRobotInfo(config) {
        if (!config) return;

        const robotName = this.panel?.querySelector('#robot-name');
        const jointCount = this.panel?.querySelector('#joint-count');
        const connectionMode = this.panel?.querySelector('#connection-mode');

        if (robotName) {
            robotName.textContent = config.robot_name || 'Unknown';
        }
        if (jointCount) {
            jointCount.textContent = config.joints?.length || '-';
        }
        if (connectionMode) {
            connectionMode.textContent = config.demo_mode ? 'Demo Mode' : (window.i18n?.t('connected') || 'Live Robot');
            connectionMode.className = `info-value ${config.demo_mode ? 'demo' : 'live'}`;
        }
    }

    /**
     * Show connecting state
     */
    setConnecting(connecting) {
        if (this.connectBtn) {
            this.connectBtn.disabled = connecting;
            this.connectBtn.textContent = connecting ? '正在连接…' : (window.i18n?.t('connect') || 'Connect');
        }
    }

    /**
     * Show error message
     */
    showError(message) {
        console.error('[ConnectionUI]', message);

        const toast = document.createElement('div');
        toast.className = 'error-toast';
        toast.textContent = message;
        document.body.appendChild(toast);

        setTimeout(() => {
            toast.classList.add('fade-out');
            setTimeout(() => toast.remove(), 300);
        }, 3000);
    }

    togglePanel() {
        if (this.panel) {
            let visible;
            if (this.panelManager) {
                visible = this.panelManager.togglePanel('floating-connection-panel', 'flex', {
                    anchorEl: this.statusIndicator,
                    align: 'center',
                    offsetX: CONNECTION_PANEL_OFFSET_X
                });
            } else {
                visible = !this.panel.classList.toggle('hidden');
            }
            this.statusIndicator?.classList.toggle('active', visible);
        }
    }

    showPanel() {
        if (this.panel) {
            if (this.panelManager) {
                this.panelManager.showPanel('floating-connection-panel', 'flex', {
                    anchorEl: this.statusIndicator,
                    align: 'center',
                    offsetX: CONNECTION_PANEL_OFFSET_X
                });
            } else {
                this.panel.classList.remove('hidden');
            }
            this.statusIndicator?.classList.add('active');
        }
    }

    hidePanel() {
        if (this.panel) {
            if (this.panelManager) {
                this.panelManager.hidePanel('floating-connection-panel');
            } else {
                this.panel.classList.add('hidden');
            }
            this.statusIndicator?.classList.remove('active');
        }
    }

    isConnected() {
        return this.robotConnection.isConnected();
    }
}
