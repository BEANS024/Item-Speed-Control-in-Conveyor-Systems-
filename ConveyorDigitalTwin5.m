%% 
classdef ConveyorDigitalTwin5 < handle
% =========================================================================
% Conveyor Belt PI Control — Digital Twin  (v5 — Arduino-aligned)
% Project: Item Speed Control in Conveyor Systems
%          for Consistent Inventory Discharge via IR Proximity Sensor
%
% HOW TO RUN:
%   >> app = ConveyorDigitalTwin5;
%
% REQUIRES: MATLAB R2019b or later (uses uifigure / uiaxes)
%
% ARDUINO ALIGNMENT (v5 changes):
%   - TARGET_GAP in ms (was 1.0 s) — now 900 ms matching Arduino constant
%   - PWM range 90–200, default 110 — matches MIN_SPEED/MAX_SPEED/DEFAULT_SPEED
%   - Asymmetric PI gains: Kp_slow/Ki_slow (e<0), Kp_fast/Ki_fast (e>=0)
%   - Integral clamped ±1000 (same as Arduino anti-windup clamp)
%   - Error in milliseconds: e(k) = measuredGap_ms - TARGET_GAP_ms
%   - Event-driven PI: only updates on IR2 exit, not every loop tick
%   - FIFO gap queue depth 10 — pairs each IR1 entry gap to its IR2 exit
%   - Feed-forward: gap < FF_TRIGGER_GAP arms a delayed FF event at +1000 ms
%   - FF only lowers speed; PI at IR2 always cancels and overrides FF
%   - Asymmetric PWM ramp: -5 per tick (slow down), +2 per tick (speed up)
%   - Belt resets to DEFAULT_SPEED + clears integral when belt empties
%
% LCD DISPLAY (v5 — 16×2 format, matches physical LCD lines):
%   Row 1:  PWM: XXX   E: ±X.X
%   Row 2:  ON:XX  GAP:X.Xs
%   Plus voltage estimate and belt speed (mm/s) below the LCD
%
% CONTROLS:
%   START / STOP / RESET  — mirrors physical push-buttons
%   Item spacing slider   — simulates inter-item entry interval at IR1
%   Kp_slow / Kp_fast sliders — asymmetric proportional gains
%   Ki_slow / Ki_fast sliders — asymmetric integral gains
%   + Add item button     — manually inject one item at IR1
%   Manual place toggle   — click belt to place item at any position
%
% PANELS:
%   Animated conveyor  — belt scrolls, items move, IR sensors flash
%   LCD readout        — mirrors physical 16×2 LCD exactly
%   Live plots         — gap (ms), PWM, PI error (30 s rolling window)
% =========================================================================

    properties (Access = private)

        % ── UI handles
        Fig
        AxBelt
        AxGap
        AxPWM
        AxError
        BtnStart, BtnStop, BtnReset, BtnAdd, BtnManual
        SldSpacing, SldKpSlow, SldKiSlow, SldKpFast, SldKiFast
        LblSpacingVal, LblKpSlowVal, LblKiSlowVal, LblKpFastVal, LblKiFastVal
        % LCD label handles
        LblLCDRow1        % "PWM: XXX   E: ±X.X"
        LblLCDRow2        % "ON:XX  GAP:X.Xs"
        LblLCDSub         % voltage + belt speed (below LCD box)
        LblIR1, LblIR2, LblStatus
        LblManualHint

        % ── Timer
        SimTimer

        % ── Simulation state (mirrors Arduino globals)
        Running         = false
        SimTime         = 0       % seconds (float — simulation clock)
        dt              = 0.05    % 50 ms loop tick

        % ── Arduino PI constants (match sketch exactly)
        TARGET_GAP      = 900     % ms — desired inter-item gap
        DEFAULT_SPEED   = 110     % PWM
        MIN_SPEED       = 90      % PWM
        MAX_SPEED       = 200     % PWM

        % Asymmetric gains — Arduino values
        Kp_slow         = 0.18    % e < 0: items too close → slow down hard
        Ki_slow         = 0.005
        Kp_fast         = 0.07    % e >= 0: items too far  → speed up gently
        Ki_fast         = 0.002

        % Anti-windup integral clamp (matches Arduino ±1000 clamp)
        INTEGRAL_CLAMP  = 1000

        % Feed-forward constants
        FF_TRIGGER_GAP  = 700     % ms — arm FF if entry gap < this
        FF_DELAY_MS     = 1000    % ms — fire FF this long after IR1
        FF_TARGET_PWM   = 120     % soft pre-emptive target

        % ── PI state
        Integral        = 0

        % ── Motor / belt
        % PWM range: 90–200 (Arduino DEFAULT=110, MIN=90, MAX=200)
        % Belt length: 390 mm (IR1 to IR2)
        % tau_motor: first-order lag to simulate L298N+TT motor inertia
        BeltLength_mm   = 390
        PWM             = 110     % current PWM (ramped)
        TargetPWM       = 110     % PI output target (before ramp)
        CurrentPWM      = 110     % ramped actual (mirrors app.PWM in ramp)
        tau_motor       = 0.4     % motor time constant (s)
        K_motor         = 0.00392 % belt speed gain: 1/255
        BeltSpeed       = 0.431   % normalised: 110/255

        % ── Items on belt  [x_position(0=IR1..1=IR2), item_id]
        Items           = zeros(0,2)
        ItemID          = 0
        ItemSpacing     = 1.5     % seconds between auto-added items (slider)
        LastItemTime    = 0
        MaterialCount   = 0       % mirrors Arduino materialCount

        % ── IR sensors
        IR1_on          = false
        IR2_on          = false
        IR1_t           = 0
        IR2_t           = 0
        IR_flash        = 0.15    % flash duration (s)

        % ── Entry gap measurement (mirrors Arduino)
        PreviousEntryTime = 0     % ms — timestamp of last IR1 trigger
        EntryGap          = 0     % ms — gap measured at IR1
        MeasuredGap       = 900   % ms — gap dequeued at IR2 for PI

        % ── FIFO gap queue (depth 10, mirrors Arduino ring buffer)
        QUEUE_SIZE      = 10
        GapQueue                  % array of gap values (ms)
        QueueHead       = 1
        QueueTail       = 1
        QueueCount      = 0

        % ── Feed-forward event queue (mirrors Arduino FFEvent struct array)
        FF_QUEUE_SIZE   = 10
        FFFireAt                  % array of fire timestamps (ms)
        FFPwm                     % array of target PWMs
        FFActive                  % logical array

        % ── Manual placement mode
        ManualMode      = false

        % ── Rolling history (30 s @ 50 ms = 600 samples)
        HTime           = []
        HGap            = []      % measured gap in ms
        HPWM            = []
        HError          = []
        MaxHist         = 600

        % ── Belt animation
        BeltOffset      = 0

    end

    % =====================================================================
    methods (Access = public)

        function app = ConveyorDigitalTwin5()
            % Initialise queue arrays
            app.GapQueue = zeros(1, app.QUEUE_SIZE);
            app.FFFireAt = zeros(1, app.FF_QUEUE_SIZE);
            app.FFPwm    = zeros(1, app.FF_QUEUE_SIZE);
            app.FFActive = false(1, app.FF_QUEUE_SIZE);

            app.buildUI();
            app.drawBelt();
            app.initPlots();

            app.SimTimer = timer( ...
                'ExecutionMode', 'fixedRate', ...
                'Period',        0.05, ...
                'TimerFcn',      @(~,~) app.step());

            app.Fig.Visible = 'on';
        end

        function delete(app)
            if ~isempty(app.SimTimer) && isvalid(app.SimTimer)
                stop(app.SimTimer);
                delete(app.SimTimer);
            end
            if ishandle(app.Fig)
                delete(app.Fig);
            end
        end

    end

    % =====================================================================
    %  FIFO QUEUE HELPERS  (mirror Arduino enqueueGap / dequeueGap)
    % =====================================================================
    methods (Access = private)

        function enqueueGap(app, gap_ms)
            if app.QueueCount < app.QUEUE_SIZE
                app.GapQueue(app.QueueTail) = gap_ms;
                app.QueueTail = mod(app.QueueTail, app.QUEUE_SIZE) + 1;
                app.QueueCount = app.QueueCount + 1;
            end
            % If full, silently drop (belt saturated — same as Arduino)
        end

        function gap = dequeueGap(app)
            if app.QueueCount > 0
                gap = app.GapQueue(app.QueueHead);
                app.QueueHead = mod(app.QueueHead, app.QUEUE_SIZE) + 1;
                app.QueueCount = app.QueueCount - 1;
            else
                gap = app.TARGET_GAP; % fallback: last item of batch, error = 0
            end
        end

    end

    % =====================================================================
    %  FEED-FORWARD EVENT HELPERS  (mirror Arduino scheduleFFEvent etc.)
    % =====================================================================
    methods (Access = private)

        function scheduleFFEvent(app, delay_ms, pwm)
            for i = 1:app.FF_QUEUE_SIZE
                if ~app.FFActive(i)
                    app.FFFireAt(i) = app.SimTime * 1000 + delay_ms;
                    app.FFPwm(i)    = pwm;
                    app.FFActive(i) = true;
                    return;
                end
            end
            % All slots full — drop silently
        end

        function processFFEvents(app)
            now_ms = app.SimTime * 1000;
            for i = 1:app.FF_QUEUE_SIZE
                if app.FFActive(i) && now_ms >= app.FFFireAt(i)
                    % Only lower speed, never raise
                    if app.FFPwm(i) < app.TargetPWM
                        app.TargetPWM = app.FFPwm(i);
                    end
                    app.FFActive(i) = false;
                end
            end
        end

        function cancelNextFFEvent(app)
            earliest = Inf;
            slot     = -1;
            for i = 1:app.FF_QUEUE_SIZE
                if app.FFActive(i) && app.FFFireAt(i) < earliest
                    earliest = app.FFFireAt(i);
                    slot     = i;
                end
            end
            if slot > 0
                app.FFActive(slot) = false;
            end
        end

        function clearAllFFEvents(app)
            app.FFActive(:) = false;
        end

    end

    % =====================================================================
    %  PI CONTROLLER  (mirrors Arduino applyPIControl exactly)
    % =====================================================================
    methods (Access = private)

        function applyPIControl(app)
            % Error in ms — same sign convention as Arduino
            e = app.MeasuredGap - app.TARGET_GAP;

            % Asymmetric gain selection
            if e < 0
                Kp = app.Kp_slow;
                Ki = app.Ki_slow;
            else
                Kp = app.Kp_fast;
                Ki = app.Ki_fast;
            end

            % Candidate output with current integral (before update)
            pTerm     = Kp * e;
            iTerm     = Ki * app.Integral;
            candidate = app.DEFAULT_SPEED + pTerm + iTerm;

            % Conditional integration — anti-windup guard
            atMin       = candidate <= app.MIN_SPEED;
            atMax       = candidate >= app.MAX_SPEED;
            windingWorse = (e < 0 && atMin) || (e > 0 && atMax);

            if ~windingWorse
                app.Integral = app.Integral + e;
                app.Integral = max(min(app.Integral, app.INTEGRAL_CLAMP), ...
                                                     -app.INTEGRAL_CLAMP);
            end

            % Recompute with updated integral
            iTerm         = Ki * app.Integral;
            app.TargetPWM = app.DEFAULT_SPEED + round(pTerm + iTerm);
            app.TargetPWM = max(min(app.TargetPWM, app.MAX_SPEED), app.MIN_SPEED);
        end

    end

    % =====================================================================
    %  MOTOR RAMP  (mirrors Arduino motorRun asymmetric ramp)
    % =====================================================================
    methods (Access = private)

        function motorRun(app)
            % Asymmetric ramp: -5 per tick (slow down), +2 per tick (speed up)
            if app.CurrentPWM > app.TargetPWM
                app.CurrentPWM = app.CurrentPWM - 5;
            elseif app.CurrentPWM < app.TargetPWM
                app.CurrentPWM = app.CurrentPWM + 2;
            end
            app.CurrentPWM = max(min(app.CurrentPWM, app.MAX_SPEED), app.MIN_SPEED);
            app.PWM        = app.CurrentPWM;

            % First-order motor lag → belt speed (normalised 0–1)
            v_target   = app.K_motor * app.PWM;
            app.BeltSpeed = app.BeltSpeed + ...
                (app.dt / app.tau_motor) * (v_target - app.BeltSpeed);
        end

    end

    % =====================================================================
    %  SIMULATION STEP  (called by timer every 50 ms)
    % =====================================================================
    methods (Access = private)

        function step(app)
            if ~app.Running; return; end

            app.SimTime = app.SimTime + app.dt;
            now_ms      = app.SimTime * 1000;  % current time in ms

            % ── Auto-add items at IR1 (auto mode only) ────────────────
            if ~app.ManualMode
                if (app.SimTime - app.LastItemTime) >= app.ItemSpacing
                    app.triggerIR1(0.0);
                    app.LastItemTime = app.SimTime;
                end
            end

            % ── Move items along belt ──────────────────────────────────
            move = app.BeltSpeed * app.dt;
            discharged = false;
            keep = true(size(app.Items,1), 1);
            for i = 1:size(app.Items,1)
                app.Items(i,1) = app.Items(i,1) + move;
                if app.Items(i,1) >= 1.0
                    keep(i)    = false;
                    discharged = true;
                end
            end
            app.Items = app.Items(keep,:);

            % ── IR2 exit event (mirrors Arduino IR2 block) ─────────────
            if discharged
                app.IR2_on = true;
                app.IR2_t  = app.SimTime;

                app.MaterialCount = max(app.MaterialCount - 1, 0);

                % Cancel matching FF before PI runs
                app.cancelNextFFEvent();

                % Dequeue the gap that belongs to this item
                app.MeasuredGap = app.dequeueGap();

                % Run PI — only here, never elsewhere
                app.applyPIControl();
            end

            % ── Fire any pending FF events ─────────────────────────────
            app.processFFEvents();

            % ── Belt empty → reset to defaults (mirrors Arduino) ───────
            if app.MaterialCount == 0
                app.TargetPWM         = app.DEFAULT_SPEED;
                app.Integral          = 0;
                app.PreviousEntryTime = 0;
                app.EntryGap          = 0;
                app.clearAllFFEvents();
                % Reset FIFO
                app.QueueHead  = 1;
                app.QueueTail  = 1;
                app.QueueCount = 0;
            end

            % ── Motor ramp ─────────────────────────────────────────────
            app.motorRun();

            % ── Belt animation ─────────────────────────────────────────
            app.BeltOffset = mod(app.BeltOffset + app.BeltSpeed * app.dt, 0.1);

            % ── IR flash timeouts ──────────────────────────────────────
            if app.IR1_on && (app.SimTime - app.IR1_t) > app.IR_flash
                app.IR1_on = false;
            end
            if app.IR2_on && (app.SimTime - app.IR2_t) > app.IR_flash
                app.IR2_on = false;
            end

            % ── Rolling history ────────────────────────────────────────
            e_ms = app.MeasuredGap - app.TARGET_GAP;
            app.HTime(end+1)  = app.SimTime;
            app.HGap(end+1)   = app.MeasuredGap;
            app.HPWM(end+1)   = app.PWM;
            app.HError(end+1) = e_ms;

            if numel(app.HTime) > app.MaxHist
                trim = numel(app.HTime) - app.MaxHist;
                app.HTime  = app.HTime(trim+1:end);
                app.HGap   = app.HGap(trim+1:end);
                app.HPWM   = app.HPWM(trim+1:end);
                app.HError = app.HError(trim+1:end);
            end

            % ── Refresh UI ─────────────────────────────────────────────
            app.drawBelt();
            app.updatePlots();
            app.updateDisplays();
        end

        % ── IR1 entry trigger (mirrors Arduino IR1 block) ──────────────
        function triggerIR1(app, x_pos)
            % Place item on belt
            app.ItemID = app.ItemID + 1;
            app.Items(end+1,:) = [x_pos, app.ItemID];

            if x_pos < 0.08
                app.IR1_on = true;
                app.IR1_t  = app.SimTime;

                countBefore = app.MaterialCount;
                app.MaterialCount = app.MaterialCount + 1;

                now_ms = app.SimTime * 1000;

                % Only measure gap if a preceding item is genuinely on belt
                if app.PreviousEntryTime > 0 && countBefore >= 1
                    app.EntryGap = now_ms - app.PreviousEntryTime;
                    app.enqueueGap(app.EntryGap);

                    % Arm FF if gap dangerously small
                    if app.EntryGap < app.FF_TRIGGER_GAP
                        app.scheduleFFEvent(app.FF_DELAY_MS, app.FF_TARGET_PWM);
                    end
                end

                app.PreviousEntryTime = now_ms;
            end
        end

    end

    % =====================================================================
    %  DRAWING
    % =====================================================================
    methods (Access = private)

        function drawBelt(app)
            ax = app.AxBelt;
            cla(ax);
            hold(ax,'on');

            % Belt body
            fill(ax,[0 1 1 0 0],[0.18 0.18 0.82 0.82 0.18], ...
                [0.22 0.22 0.22],'EdgeColor',[0.08 0.08 0.08],'LineWidth',1.5);

            % Scrolling stripes
            for i = 0:11
                x = mod(i/12 + app.BeltOffset, 1.0);
                plot(ax,[x x],[0.18 0.82],'-','Color',[0.32 0.32 0.32],'LineWidth',0.8);
            end

            % Items (yellow boxes with ID labels)
            for i = 1:size(app.Items,1)
                xp = min(max(app.Items(i,1), 0.03), 0.97);
                fill(ax, xp+[-0.035 0.035 0.035 -0.035 -0.035], ...
                         [0.27 0.27 0.73 0.73 0.27], ...
                    [0.97 0.78 0.12],'EdgeColor',[0.65 0.50 0.0],'LineWidth',1.2);
                text(ax, xp, 0.50, num2str(app.Items(i,2)), ...
                    'HorizontalAlignment','center','FontSize',8, ...
                    'FontWeight','bold','Color',[0.15 0.07 0.0]);
                % Position label in mm below item
                pos_mm = round(xp * app.BeltLength_mm);
                text(ax, xp, 0.22, sprintf('%dmm', pos_mm), ...
                    'HorizontalAlignment','center','FontSize',6, ...
                    'Color',[0.70 0.55 0.10]);
            end

            % IR1 sensor (entry)
            c1 = [0.20 0.85 0.20];
            if app.IR1_on; c1 = [1.00 0.35 0.05]; end
            plot(ax,[0.05 0.05],[0.00 0.18],'-','Color',c1,'LineWidth',3);
            plot(ax,[0.05 0.05],[0.82 1.00],'-','Color',c1,'LineWidth',3);
            if app.IR1_on
                plot(ax,[0.05 0.05],[0.18 0.82],'--','Color',[1 0.5 0.1],'LineWidth',1);
            end
            text(ax,0.05,1.10,'IR1','HorizontalAlignment','center', ...
                'FontSize',9,'FontWeight','bold','Color',c1);

            % IR2 sensor (exit)
            c2 = [0.20 0.85 0.20];
            if app.IR2_on; c2 = [1.00 0.35 0.05]; end
            plot(ax,[0.95 0.95],[0.00 0.18],'-','Color',c2,'LineWidth',3);
            plot(ax,[0.95 0.95],[0.82 1.00],'-','Color',c2,'LineWidth',3);
            if app.IR2_on
                plot(ax,[0.95 0.95],[0.18 0.82],'--','Color',[1 0.5 0.1],'LineWidth',1);
            end
            text(ax,0.95,1.10,'IR2','HorizontalAlignment','center', ...
                'FontSize',9,'FontWeight','bold','Color',c2);

            % Belt direction arrow
            plot(ax,[0.42 0.58],[1.18 1.18],'->','Color',[0.55 0.65 0.90], ...
                'LineWidth',1.5,'MarkerSize',6);
            text(ax,0.50,1.25,'Belt direction','HorizontalAlignment','center', ...
                'FontSize',8,'Color',[0.55 0.65 0.90]);

            % Manual mode overlay
            if app.ManualMode
                rectangle(ax,'Position',[0 0.18 1.0 0.64], ...
                    'EdgeColor',[0.98 0.72 0.10],'LineWidth',1.5, ...
                    'LineStyle','--','FaceColor','none');
                text(ax,0.50,0.93,'click to place item', ...
                    'HorizontalAlignment','center','FontSize',8, ...
                    'Color',[0.98 0.72 0.10],'FontAngle','italic');
            end

            % PWM speed bar (scaled to MIN–MAX operating range)
            sf = (app.PWM - app.MIN_SPEED) / (app.MAX_SPEED - app.MIN_SPEED);
            sf = max(min(sf, 1), 0);
            bc = [0.15 + 0.65*sf, 0.70 - 0.45*sf, 0.18];
            fill(ax,[0 sf sf 0 0],[-0.20 -0.20 -0.10 -0.10 -0.20], ...
                bc,'EdgeColor','none');
            rectangle(ax,'Position',[0 -0.20 1 0.10], ...
                'EdgeColor',[0.45 0.45 0.45],'FaceColor','none','LineWidth',0.8);
            % Voltage estimate: PWM/255 × 13.7 V − 2 V L298N drop
            v_est = max((app.PWM/255) * 13.7 - 2.0, 0);
            % Belt speed in mm/s
            spd_mm = app.BeltSpeed * 255 * app.K_motor * 1000 * app.BeltLength_mm;
            text(ax,0.50,-0.15, ...
                sprintf('PWM %d  |  ~%.1fV  |  ~%.0f mm/s  |  Belt: %d mm', ...
                    app.PWM, v_est, spd_mm, app.BeltLength_mm), ...
                'HorizontalAlignment','center','FontSize',8,'Color',[0.80 0.80 0.80]);

            hold(ax,'off');
            xlim(ax,[-0.12 1.12]);
            ylim(ax,[-0.28 1.35]);
            axis(ax,'off');
            drawnow limitrate;
        end

        % ── Live plots ────────────────────────────────────────────────────
        function updatePlots(app)
            if numel(app.HTime) < 2; return; end
            t    = app.HTime;
            tmin = max(t(end)-30, t(1));
            twin = [tmin, t(end)];

            % Gap plot (ms) — setpoint line at TARGET_GAP
            cla(app.AxGap);
            plot(app.AxGap, t, app.HGap, 'Color',[0.25 0.55 1.0],'LineWidth',1.4);
            hold(app.AxGap,'on');
            yline(app.AxGap, app.TARGET_GAP, 'r--', 'LineWidth',1.2);
            yline(app.AxGap, app.FF_TRIGGER_GAP, '--', ...
                  'Color',[0.98 0.72 0.10], 'LineWidth',0.9);
            hold(app.AxGap,'off');
            xlim(app.AxGap, twin);
            ylim(app.AxGap, [0 3000]);
            ylabel(app.AxGap,'Gap (ms)');
            title(app.AxGap, ...
                sprintf('Inter-item gap  |  red = target %d ms  |  amber = FF trigger %d ms', ...
                    app.TARGET_GAP, app.FF_TRIGGER_GAP), 'FontSize',8);
            grid(app.AxGap,'on');

            % PWM plot — show DEFAULT, MIN, MAX reference lines
            cla(app.AxPWM);
            plot(app.AxPWM, t, app.HPWM,'Color',[0.65 0.25 0.95],'LineWidth',1.4);
            hold(app.AxPWM,'on');
            yline(app.AxPWM, app.DEFAULT_SPEED, 'k--',  'LineWidth',1.0);
            yline(app.AxPWM, app.MIN_SPEED,     '--','Color',[0.8 0.3 0.3],'LineWidth',0.8);
            yline(app.AxPWM, app.MAX_SPEED,     '--','Color',[0.8 0.3 0.3],'LineWidth',0.8);
            yline(app.AxPWM, app.FF_TARGET_PWM, '--','Color',[0.98 0.72 0.10],'LineWidth',0.8);
            hold(app.AxPWM,'off');
            xlim(app.AxPWM, twin);
            ylim(app.AxPWM, [75 215]);
            ylabel(app.AxPWM,'PWM');
            title(app.AxPWM, ...
                sprintf('PWM output  |  default=%d  min=%d  max=%d  FF=%d', ...
                    app.DEFAULT_SPEED, app.MIN_SPEED, app.MAX_SPEED, app.FF_TARGET_PWM), ...
                'FontSize',8);
            grid(app.AxPWM,'on');

            % Error plot (ms)
            cla(app.AxError);
            % Colour error line: orange for positive (too slow), blue for negative
            e_data = app.HError;
            plot(app.AxError, t, e_data,'Color',[0.95 0.55 0.10],'LineWidth',1.4);
            hold(app.AxError,'on');
            yline(app.AxError, 0, 'k--','LineWidth',1.0);
            hold(app.AxError,'off');
            xlim(app.AxError, twin);
            ylim(app.AxError, [-1500 1500]);
            ylabel(app.AxError,'Error (ms)');
            xlabel(app.AxError,'Time (s)');
            title(app.AxError,'PI error  e(k) = measuredGap − TARGET_GAP  (ms)','FontSize',8);
            grid(app.AxError,'on');
        end

        % ── LCD display (mirrors physical 16×2 LCD) ───────────────────────
        function updateDisplays(app)
            % Row 1 of physical LCD:  "PWM:XXX  E:±X.X"
            e_ms    = app.MeasuredGap - app.TARGET_GAP;
            e_tenth = e_ms / 100;  % matches Arduino /100 scaling
            row1    = sprintf('PWM:%-3d  E:%+.1f', round(app.PWM), e_tenth);
            app.LblLCDRow1.Text = row1;

            % Row 2 of physical LCD:  "ON:XX  GAP:X.Xs"
            gap_s  = app.MeasuredGap / 1000;
            row2   = sprintf('ON:%-2d   GAP:%.2fs', app.MaterialCount, gap_s);
            app.LblLCDRow2.Text = row2;

            % Sub-line: voltage, belt speed, asymmetric gains in use
            v_est   = max((app.PWM/255) * 13.7 - 2.0, 0);
            spd_mm  = app.BeltSpeed * 255 * app.K_motor * 1000 * app.BeltLength_mm;
            if e_ms < 0
                gain_str = sprintf('Kp=%.2f  Ki=%.3f  [SLOW]', app.Kp_slow, app.Ki_slow);
            else
                gain_str = sprintf('Kp=%.2f  Ki=%.3f  [FAST]', app.Kp_fast, app.Ki_fast);
            end
            app.LblLCDSub.Text = sprintf('%.1fV  |  %.0f mm/s  |  %s', ...
                v_est, spd_mm, gain_str);

            % Colour-code Row1 label by error magnitude
            if abs(e_ms) < 50
                app.LblLCDRow1.FontColor = [0.22 1.00 0.22];  % green: on-target
                app.LblLCDRow2.FontColor = [0.22 1.00 0.22];
            elseif e_ms < 0
                app.LblLCDRow1.FontColor = [0.30 0.70 1.00];  % blue: too fast
                app.LblLCDRow2.FontColor = [0.30 0.70 1.00];
            else
                app.LblLCDRow1.FontColor = [1.00 0.55 0.15];  % orange: too slow
                app.LblLCDRow2.FontColor = [1.00 0.55 0.15];
            end

            % IR status indicators
            if app.IR1_on
                app.LblIR1.Text      = 'IR1  ● TRIGGERED';
                app.LblIR1.FontColor = [1.0 0.4 0.1];
            else
                app.LblIR1.Text      = 'IR1  ○ waiting';
                app.LblIR1.FontColor = [0.35 0.80 0.35];
            end
            if app.IR2_on
                app.LblIR2.Text      = 'IR2  ● TRIGGERED';
                app.LblIR2.FontColor = [1.0 0.4 0.1];
            else
                app.LblIR2.Text      = 'IR2  ○ waiting';
                app.LblIR2.FontColor = [0.35 0.80 0.35];
            end
        end

        function initPlots(app)
            axes_list = {app.AxGap, app.AxPWM, app.AxError};
            ylims     = {[0 3000], [75 215], [-1500 1500]};
            ttls      = { ...
                sprintf('Inter-item gap  |  red = target %d ms', app.TARGET_GAP), ...
                sprintf('PWM output  |  default=%d  min=%d  max=%d', ...
                    app.DEFAULT_SPEED, app.MIN_SPEED, app.MAX_SPEED), ...
                'PI error  e(k) = measuredGap − TARGET_GAP  (ms)'};
            ylbls = {'Gap (ms)', 'PWM', 'Error (ms)'};
            for i = 1:3
                ax = axes_list{i};
                xlim(ax,[0 30]); ylim(ax,ylims{i});
                title(ax, ttls{i}, 'FontSize',8);
                ylabel(ax, ylbls{i});
                grid(ax,'on');
            end
            xlabel(app.AxError,'Time (s)');
        end

    end

    % =====================================================================
    %  UI CONSTRUCTION
    % =====================================================================
    methods (Access = private)

        function buildUI(app)

            % ── Figure ────────────────────────────────────────────────────
            app.Fig = uifigure( ...
                'Name',     'Conveyor PI Control  —  Digital Twin v5', ...
                'Position', [60 60 1220 790], ...
                'Color',    [0.11 0.11 0.13], ...
                'Visible',  'off');
            app.Fig.DeleteFcn = @(~,~) app.delete();

            % ── Header ────────────────────────────────────────────────────
            uilabel(app.Fig,'Text', ...
                'CONVEYOR BELT PI CONTROL  —  DIGITAL TWIN  v5', ...
                'Position',[20 758 960 24],'FontSize',14,'FontWeight','bold', ...
                'FontColor',[0.92 0.92 0.92]);
            uilabel(app.Fig,'Text', ...
                ['Item Speed Control for Consistent Inventory Discharge  |  ' ...
                 'Belt: 390 mm  |  Target gap: 900 ms  |  PWM: 90–200 (default 110)  |  ' ...
                 'Asymmetric PI  |  FIFO depth: 10'], ...
                'Position',[20 740 1100 18],'FontSize',9,'FontColor',[0.58 0.58 0.70]);

            % ── Conveyor panel ────────────────────────────────────────────
            pBelt = uipanel(app.Fig,'Title','Conveyor Belt', ...
                'Position',[15 448 760 285], ...
                'BackgroundColor',[0.11 0.11 0.13], ...
                'ForegroundColor',[0.80 0.80 0.80],'FontWeight','bold');

            app.AxBelt = uiaxes(pBelt,'Position',[8 8 744 232], ...
                'Color',[0.07 0.07 0.09],'XColor','none','YColor','none', ...
                'Interactions',[]);

            app.LblManualHint = uilabel(pBelt, ...
                'Text','▲  Manual mode ON — click belt to place an item', ...
                'Position',[8 242 744 18], ...
                'FontSize',9,'FontWeight','bold', ...
                'FontColor',[0.98 0.75 0.20], ...
                'HorizontalAlignment','center', ...
                'Visible','off');

            % ── Control panel ─────────────────────────────────────────────
            pCtrl = uipanel(app.Fig,'Title','Control Panel', ...
                'Position',[15 12 760 428], ...
                'BackgroundColor',[0.11 0.11 0.13], ...
                'ForegroundColor',[0.80 0.80 0.80],'FontWeight','bold');

            % Buttons row
            app.BtnStart = uibutton(pCtrl,'Text','▶  START', ...
                'Position',[14 380 110 36],'FontSize',11,'FontWeight','bold', ...
                'ButtonPushedFcn',@(s,e) app.cbStart(s,e));
            app.BtnStop = uibutton(pCtrl,'Text','■  STOP', ...
                'Position',[134 380 110 36],'FontSize',11,'FontWeight','bold', ...
                'ButtonPushedFcn',@(s,e) app.cbStop(s,e));
            app.BtnReset = uibutton(pCtrl,'Text','↺  RESET', ...
                'Position',[254 380 110 36],'FontSize',11,'FontWeight','bold', ...
                'ButtonPushedFcn',@(s,e) app.cbReset(s,e));
            app.BtnAdd = uibutton(pCtrl,'Text','+ Add at IR1', ...
                'Position',[374 380 130 36],'FontSize',11,'FontWeight','bold', ...
                'BackgroundColor',[0.18 0.45 0.78],'FontColor',[1 1 1], ...
                'ButtonPushedFcn',@(s,e) app.cbAdd(s,e));
            app.BtnManual = uibutton(pCtrl,'Text','✎  Manual: OFF', ...
                'Position',[514 380 156 36],'FontSize',10,'FontWeight','bold', ...
                'BackgroundColor',[0.88 0.88 0.88],'FontColor',[0 0 0], ...
                'ButtonPushedFcn',@(s,e) app.cbManual(s,e));

            % ── Sliders ───────────────────────────────────────────────────
            % Item spacing
            uilabel(pCtrl,'Text','Item spacing  (entry interval at IR1)', ...
                'Position',[14 338 500 18],'FontWeight','bold','FontColor',[0.88 0.88 0.88]);
            app.SldSpacing = uislider(pCtrl,'Limits',[0.3 4.0],'Value',1.5, ...
                'Position',[14 322 600 3], ...
                'ValueChangedFcn', @(s,e) app.cbSpacing(s,e), ...
                'ValueChangingFcn',@(s,e) app.cbSpacing(s,e));
            app.LblSpacingVal = uilabel(pCtrl,'Text','1.5 s', ...
                'Position',[626 315 90 22],'FontSize',13,'FontWeight','bold', ...
                'FontColor',[0.98 0.82 0.28]);

            % Kp_slow
            uilabel(pCtrl,'Text','Kp slow  (e < 0 : items too close — decelerate)', ...
                'Position',[14 278 420 18],'FontWeight','bold','FontColor',[0.88 0.88 0.88]);
            app.SldKpSlow = uislider(pCtrl,'Limits',[0.01 0.80],'Value',0.18, ...
                'Position',[14 262 600 3], ...
                'ValueChangedFcn', @(s,e) app.cbKpSlow(s,e), ...
                'ValueChangingFcn',@(s,e) app.cbKpSlow(s,e));
            app.LblKpSlowVal = uilabel(pCtrl,'Text','0.18', ...
                'Position',[626 255 90 22],'FontSize',13,'FontWeight','bold', ...
                'FontColor',[0.45 0.80 1.00]);

            % Ki_slow
            uilabel(pCtrl,'Text','Ki slow', ...
                'Position',[14 218 200 18],'FontWeight','bold','FontColor',[0.88 0.88 0.88]);
            app.SldKiSlow = uislider(pCtrl,'Limits',[0.000 0.030],'Value',0.005, ...
                'Position',[14 202 600 3], ...
                'ValueChangedFcn', @(s,e) app.cbKiSlow(s,e), ...
                'ValueChangingFcn',@(s,e) app.cbKiSlow(s,e));
            app.LblKiSlowVal = uilabel(pCtrl,'Text','0.005', ...
                'Position',[626 195 90 22],'FontSize',13,'FontWeight','bold', ...
                'FontColor',[0.45 0.80 1.00]);

            % Kp_fast
            uilabel(pCtrl,'Text','Kp fast  (e ≥ 0 : items too far — accelerate gently)', ...
                'Position',[14 158 420 18],'FontWeight','bold','FontColor',[0.88 0.88 0.88]);
            app.SldKpFast = uislider(pCtrl,'Limits',[0.01 0.40],'Value',0.07, ...
                'Position',[14 142 600 3], ...
                'ValueChangedFcn', @(s,e) app.cbKpFast(s,e), ...
                'ValueChangingFcn',@(s,e) app.cbKpFast(s,e));
            app.LblKpFastVal = uilabel(pCtrl,'Text','0.07', ...
                'Position',[626 135 90 22],'FontSize',13,'FontWeight','bold', ...
                'FontColor',[0.45 0.80 1.00]);

            % Ki_fast
            uilabel(pCtrl,'Text','Ki fast', ...
                'Position',[14 98 200 18],'FontWeight','bold','FontColor',[0.88 0.88 0.88]);
            app.SldKiFast = uislider(pCtrl,'Limits',[0.000 0.015],'Value',0.002, ...
                'Position',[14 82 600 3], ...
                'ValueChangedFcn', @(s,e) app.cbKiFast(s,e), ...
                'ValueChangingFcn',@(s,e) app.cbKiFast(s,e));
            app.LblKiFastVal = uilabel(pCtrl,'Text','0.002', ...
                'Position',[626 75 90 22],'FontSize',13,'FontWeight','bold', ...
                'FontColor',[0.45 0.80 1.00]);

            % PI law note (updated formula)
            uilabel(pCtrl,'Text', ...
                'PI law:  u(k) = DEFAULT_SPEED + Kp(e)·e(k) + Ki(e)·Σe(k)   |   e(k) = measuredGap − 900 ms   |   gains selected by sign(e)', ...
                'Position',[14 44 720 20],'FontSize',9,'FontAngle','italic', ...
                'FontColor',[0.48 0.72 0.48]);

            % IR status + system status
            app.LblIR1 = uilabel(pCtrl,'Text','IR1  ○ waiting', ...
                'Position',[14 16 220 22],'FontSize',12,'FontWeight','bold', ...
                'FontColor',[0.35 0.80 0.35]);
            app.LblIR2 = uilabel(pCtrl,'Text','IR2  ○ waiting', ...
                'Position',[280 16 220 22],'FontSize',12,'FontWeight','bold', ...
                'FontColor',[0.35 0.80 0.35]);
            app.LblStatus = uilabel(pCtrl,'Text','○ READY', ...
                'Position',[560 16 180 22],'FontSize',13,'FontWeight','bold', ...
                'FontColor',[0.55 0.55 0.55]);

            % ── LCD panel ─────────────────────────────────────────────────
            % Styled to resemble a physical 16×2 LCD (dark green on black)
            pLCD = uipanel(app.Fig,'Title', ...
                '16×2 LCD  —  mirrors physical display', ...
                'Position',[785 580 420 153], ...
                'BackgroundColor',[0.02 0.06 0.02], ...
                'ForegroundColor',[0.35 0.88 0.35],'FontWeight','bold');

            % LCD character-cell background
            uipanel(pLCD,'Position',[10 40 396 92], ...
                'BackgroundColor',[0.02 0.09 0.02],'BorderType','line', ...
                'BorderColor',[0.20 0.60 0.20]);

            % Row 1: "PWM:XXX  E:±X.X"
            uilabel(pLCD,'Text','Row 1', ...
                'Position',[12 118 80 14],'FontSize',8,'FontColor',[0.25 0.55 0.25]);
            app.LblLCDRow1 = uilabel(pLCD, ...
                'Text',     'PWM:110  E:+0.0', ...
                'Position', [12 92 396 28], ...
                'FontSize', 18, 'FontWeight','bold', ...
                'FontName', 'Courier New', ...
                'FontColor',[0.22 1.00 0.22]);

            % Row 2: "ON:XX  GAP:X.XXs"
            uilabel(pLCD,'Text','Row 2', ...
                'Position',[12 74 80 14],'FontSize',8,'FontColor',[0.25 0.55 0.25]);
            app.LblLCDRow2 = uilabel(pLCD, ...
                'Text',     'ON:0    GAP:0.90s', ...
                'Position', [12 46 396 28], ...
                'FontSize', 18, 'FontWeight','bold', ...
                'FontName', 'Courier New', ...
                'FontColor',[0.22 1.00 0.22]);

            % Sub-line: voltage, belt speed, active gains
            app.LblLCDSub = uilabel(pLCD, ...
                'Text',     '0.0V  |  0 mm/s  |  Kp=0.18  Ki=0.005  [SLOW]', ...
                'Position', [12 14 396 22], ...
                'FontSize', 9, ...
                'FontColor',[0.35 0.70 0.35]);

            % ── Live plots panel ──────────────────────────────────────────
            pPlots = uipanel(app.Fig,'Title','Live PI Signals  (30 s rolling window)', ...
                'Position',[785 12 420 560], ...
                'BackgroundColor',[0.09 0.09 0.12], ...
                'ForegroundColor',[0.80 0.80 0.80],'FontWeight','bold');

            h = 158; g = 10;
            app.AxGap = uiaxes(pPlots,'Position',[10 g*2+h*2 398 h], ...
                'Color',[0.06 0.06 0.08],'XColor',[0.58 0.58 0.58], ...
                'YColor',[0.58 0.58 0.58]);
            app.AxPWM = uiaxes(pPlots,'Position',[10 g+h 398 h], ...
                'Color',[0.06 0.06 0.08],'XColor',[0.58 0.58 0.58], ...
                'YColor',[0.58 0.58 0.58]);
            app.AxError = uiaxes(pPlots,'Position',[10 g 398 h], ...
                'Color',[0.06 0.06 0.08],'XColor',[0.58 0.58 0.58], ...
                'YColor',[0.58 0.58 0.58]);
        end

    end

    % =====================================================================
    %  BUTTON CALLBACKS
    % =====================================================================
    methods (Access = private)

        function cbStart(app,~,~)
            if app.Running; return; end
            app.Running = true;
            app.BtnStart.BackgroundColor = [0.15 0.65 0.15];
            app.BtnStart.FontColor       = [1 1 1];
            app.BtnStop.BackgroundColor  = [0.88 0.88 0.88];
            app.BtnStop.FontColor        = [0 0 0];
            app.LblStatus.Text           = '● RUNNING';
            app.LblStatus.FontColor      = [0.10 0.70 0.10];
            start(app.SimTimer);
        end

        function cbStop(app,~,~)
            if ~app.Running; return; end
            app.Running = false;
            stop(app.SimTimer);
            app.BtnStop.BackgroundColor  = [0.75 0.15 0.15];
            app.BtnStop.FontColor        = [1 1 1];
            app.BtnStart.BackgroundColor = [0.88 0.88 0.88];
            app.BtnStart.FontColor       = [0 0 0];
            app.LblStatus.Text           = '■ HALTED';
            app.LblStatus.FontColor      = [0.80 0.15 0.15];
        end

        function cbReset(app,~,~)
            app.cbStop();
            % Reset all state — mirrors Arduino resetSystem()
            app.SimTime           = 0;
            app.Integral          = 0;
            app.PWM               = app.DEFAULT_SPEED;
            app.TargetPWM         = app.DEFAULT_SPEED;
            app.CurrentPWM        = app.DEFAULT_SPEED;
            app.BeltSpeed         = app.DEFAULT_SPEED * app.K_motor;
            app.Items             = zeros(0,2);
            app.ItemID            = 0;
            app.LastItemTime      = 0;
            app.MaterialCount     = 0;
            app.PreviousEntryTime = 0;
            app.EntryGap          = 0;
            app.MeasuredGap       = app.TARGET_GAP;
            app.IR1_on = false; app.IR2_on = false;
            % Reset FIFO
            app.GapQueue  = zeros(1, app.QUEUE_SIZE);
            app.QueueHead = 1;
            app.QueueTail = 1;
            app.QueueCount = 0;
            % Reset FF queue
            app.clearAllFFEvents();
            % Reset history
            app.HTime=[]; app.HGap=[]; app.HPWM=[]; app.HError=[];
            app.BeltOffset = 0;
            % Return to auto mode
            if app.ManualMode
                app.ManualMode = false;
                app.BtnManual.Text            = '✎  Manual: OFF';
                app.BtnManual.BackgroundColor = [0.88 0.88 0.88];
                app.BtnManual.FontColor       = [0 0 0];
                app.LblManualHint.Visible     = 'off';
                app.AxBelt.ButtonDownFcn      = '';
            end
            app.LblStatus.Text           = '○ READY';
            app.LblStatus.FontColor      = [0.55 0.55 0.55];
            app.BtnStart.BackgroundColor = [0.88 0.88 0.88];
            app.BtnStart.FontColor       = [0 0 0];
            app.BtnStop.BackgroundColor  = [0.88 0.88 0.88];
            app.BtnStop.FontColor        = [0 0 0];
            app.drawBelt();
            cla(app.AxGap); cla(app.AxPWM); cla(app.AxError);
            app.initPlots();
            app.updateDisplays();
        end

        function cbAdd(app,~,~)
            app.triggerIR1(0.0);
        end

        function cbManual(app,~,~)
            app.ManualMode = ~app.ManualMode;
            if app.ManualMode
                app.BtnManual.Text            = '✎  Manual: ON';
                app.BtnManual.BackgroundColor = [0.70 0.45 0.05];
                app.BtnManual.FontColor       = [1 1 1];
                app.LblManualHint.Visible     = 'on';
                app.AxBelt.ButtonDownFcn = @(~,evt) app.cbBeltClick(evt);
            else
                app.BtnManual.Text            = '✎  Manual: OFF';
                app.BtnManual.BackgroundColor = [0.88 0.88 0.88];
                app.BtnManual.FontColor       = [0 0 0];
                app.LblManualHint.Visible     = 'off';
                app.AxBelt.ButtonDownFcn      = '';
            end
        end

        function cbBeltClick(app, evt)
            if ~app.Running; return; end
            cp     = evt.IntersectionPoint;
            x_norm = (cp(1) + 0.12) / 1.24;
            x_norm = max(min(x_norm, 0.98), 0.0);
            if cp(2) >= 0.10 && cp(2) <= 0.90
                app.triggerIR1(x_norm);
            end
        end

        function cbSpacing(app,src,~)
            app.ItemSpacing        = src.Value;
            app.LblSpacingVal.Text = sprintf('%.1f s', src.Value);
        end

        function cbKpSlow(app,src,~)
            app.Kp_slow            = src.Value;
            app.LblKpSlowVal.Text  = sprintf('%.3f', src.Value);
        end

        function cbKiSlow(app,src,~)
            app.Ki_slow            = src.Value;
            app.LblKiSlowVal.Text  = sprintf('%.4f', src.Value);
        end

        function cbKpFast(app,src,~)
            app.Kp_fast            = src.Value;
            app.LblKpFastVal.Text  = sprintf('%.3f', src.Value);
        end

        function cbKiFast(app,src,~)
            app.Ki_fast            = src.Value;
            app.LblKiFastVal.Text  = sprintf('%.4f', src.Value);
        end

    end
end
