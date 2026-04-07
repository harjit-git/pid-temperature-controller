classdef Project_appD_v2_final < matlab.apps.AppBase

    properties (Access = public)
        UIFigure                matlab.ui.Figure
        UIAxes                  matlab.ui.control.UIAxes
        CurrentTempLabel        matlab.ui.control.Label
        PWMLabel                matlab.ui.control.Label
        COMPortEditFieldLabel   matlab.ui.control.Label
        COMPortEditField        matlab.ui.control.EditField
        BaudRateEditFieldLabel  matlab.ui.control.Label
        BaudRateEditField       matlab.ui.control.NumericEditField
        SetpointEditFieldLabel  matlab.ui.control.Label
        SetpointEditField       matlab.ui.control.NumericEditField
        ConnectButton           matlab.ui.control.Button
        DisconnectButton        matlab.ui.control.Button
        SendSetpointButton      matlab.ui.control.Button
        SendPIDButton           matlab.ui.control.Button
        StatusTextAreaLabel     matlab.ui.control.Label
        StatusTextArea          matlab.ui.control.TextArea
        KpSliderLabel           matlab.ui.control.Label
        KpSlider                matlab.ui.control.Slider
        KpValueLabel            matlab.ui.control.Label
        KiSliderLabel           matlab.ui.control.Label
        KiSlider                matlab.ui.control.Slider
        KiValueLabel            matlab.ui.control.Label
        KdSliderLabel           matlab.ui.control.Label
        KdSlider                matlab.ui.control.Slider
        KdValueLabel            matlab.ui.control.Label
    end

    properties (Access = private)
        serialPort = []
        timer_obj  = []
        time_data  = []
        temp_data  = []
        sp_data    = []
        t_start
    end

    % ── Private helpers ─────────────────────────────────────
    methods (Access = private)

        function connectSerial(app)
            port = strtrim(app.COMPortEditField.Value);
            baud = app.BaudRateEditField.Value;
            if isempty(port)
                app.logStatus('ERROR: Enter COM port first'); return
            end
            app.disconnectSerial();
            try
                app.serialPort = serialport(port, baud);
                configureTerminator(app.serialPort, "CR/LF");
                app.time_data = [];
                app.temp_data = [];
                app.sp_data   = [];
                app.t_start   = tic;
                app.logStatus(['Connected: ' port ' @ ' num2str(baud)]);
                app.ConnectButton.Enable    = 'off';
                app.DisconnectButton.Enable = 'on';
                app.timer_obj = timer( ...
                    'ExecutionMode', 'fixedRate', ...
                    'Period',        0.25, ...
                    'TimerFcn',      @(~,~) app.readSerial());
                start(app.timer_obj);
            catch e
                app.logStatus(['ERROR: ' e.message]);
            end
        end

        function disconnectSerial(app)
            if ~isempty(app.timer_obj) && isvalid(app.timer_obj)
                stop(app.timer_obj);
                delete(app.timer_obj);
                app.timer_obj = [];
            end
            if ~isempty(app.serialPort)
                delete(app.serialPort);
                app.serialPort = [];
            end
            app.logStatus('Disconnected.');
            app.ConnectButton.Enable    = 'on';
            app.DisconnectButton.Enable = 'off';
        end

        function sendSetpoint(app)
            if isempty(app.serialPort)
                app.logStatus('Not connected.'); return
            end
            sp = app.SetpointEditField.Value;
            if sp < 5 || sp > 45
                app.logStatus('Setpoint must be 5-45 C'); return
            end
            writeline(app.serialPort, sprintf('%.1f', sp));
            app.logStatus(['Setpoint sent: ' num2str(sp) ' C']);
        end

        function sendPID(app)
            if isempty(app.serialPort)
                app.logStatus('Not connected.'); return
            end
            kp = app.KpSlider.Value;
            ki = app.KiSlider.Value;
            kd = app.KdSlider.Value;
            writeline(app.serialPort, sprintf('K:%.2f,%.3f,%.2f', kp, ki, kd));
            app.logStatus(sprintf('PID sent Kp=%.1f Ki=%.3f Kd=%.1f', kp, ki, kd));
        end

        function readSerial(app)
            try
                while app.serialPort.NumBytesAvailable > 0
                    line = strtrim(readline(app.serialPort));
                    if isempty(line),            continue, end
                    if startsWith(line, '='),    continue, end
                    if startsWith(line, 'temp'), continue, end
                    if startsWith(line, '>>'),   continue, end

                    % Show PWM debug line in label
                    if startsWith(line, 'PWM')
                        app.PWMLabel.Text = line;
                        continue
                    end

                    % CSV: temp,setpoint,control,pwm
                    parts = strsplit(line, ',');
                    if numel(parts) < 2, continue, end
                    T  = str2double(parts{1});
                    SP = str2double(parts{2});
                    if isnan(T) || isnan(SP), continue, end

                    t = toc(app.t_start);
                    app.time_data(end+1) = t;
                    app.temp_data(end+1) = T;
                    app.sp_data(end+1)   = SP;

                    % Large colour-coded temperature
                    app.CurrentTempLabel.Text = sprintf('%.1f °C', T);
                    err = abs(T - SP);
                    if err <= 0.5
                        app.CurrentTempLabel.FontColor = [0.10 0.75 0.10];
                    elseif T > SP
                        app.CurrentTempLabel.FontColor = [0.93 0.25 0.20];
                    else
                        app.CurrentTempLabel.FontColor = [0.20 0.50 0.95];
                    end

                    % Plot
                    cla(app.UIAxes);
                    plot(app.UIAxes, app.time_data, app.temp_data, ...
                         'b-',  'LineWidth', 1.8, 'DisplayName', 'Temperature');
                    hold(app.UIAxes, 'on');
                    plot(app.UIAxes, app.time_data, app.sp_data, ...
                         'r--', 'LineWidth', 1.4, 'DisplayName', 'Setpoint');
                    hold(app.UIAxes, 'off');
                    legend(app.UIAxes, 'Location', 'northeast');
                    app.UIAxes.XLabel.String = 'Time (s)';
                    app.UIAxes.YLabel.String = 'Temperature (°C)';
                    app.UIAxes.Title.String  = 'Temperature vs Time';
                    app.UIAxes.YLim = [0 60];
                    grid(app.UIAxes, 'on');
                    drawnow limitrate;
                end
            catch e
                app.logStatus(['Read error: ' e.message]);
            end
        end

        function logStatus(app, msg)
            existing = app.StatusTextArea.Value;
            if ischar(existing), existing = {existing}; end
            newLines = [existing; {msg}];
            if numel(newLines) > 8
                newLines = newLines(end-7:end);
            end
            app.StatusTextArea.Value = newLines;
        end

        function updateSliderLabels(app)
            app.KpValueLabel.Text = sprintf('Kp = %.1f',  app.KpSlider.Value);
            app.KiValueLabel.Text = sprintf('Ki = %.3f',  app.KiSlider.Value);
            app.KdValueLabel.Text = sprintf('Kd = %.1f',  app.KdSlider.Value);
        end

    end

    % ── Component creation ───────────────────────────────────
    methods (Access = private)

        function createComponents(app)

            app.UIFigure = uifigure('Visible','off');
            app.UIFigure.Position = [100 100 996 565];
            app.UIFigure.Name = 'PID Temperature Controller - EP3BB3';
            app.UIFigure.CloseRequestFcn = @(~,~) delete(app);

            % Plot
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes,  'Temperature vs Time')
            xlabel(app.UIAxes, 'Time (s)')
            ylabel(app.UIAxes, 'Temperature (°C)')
            app.UIAxes.Position = [14 236 680 316];

            % Large current temp
            app.CurrentTempLabel = uilabel(app.UIFigure);
            app.CurrentTempLabel.Position = [710 390 270 65];
            app.CurrentTempLabel.Text = '-- °C';
            app.CurrentTempLabel.FontSize   = 34;
            app.CurrentTempLabel.FontWeight = 'bold';
            app.CurrentTempLabel.FontColor  = [0.6 0.6 0.6];
            app.CurrentTempLabel.HorizontalAlignment = 'center';

            % PWM status
            app.PWMLabel = uilabel(app.UIFigure);
            app.PWMLabel.Position = [710 360 270 24];
            app.PWMLabel.Text = 'PWM: --';
            app.PWMLabel.HorizontalAlignment = 'center';

            % COM Port
            app.COMPortEditFieldLabel = uilabel(app.UIFigure);
            app.COMPortEditFieldLabel.Position = [14 204 80 22];
            app.COMPortEditFieldLabel.Text = 'COM Port:';
            app.COMPortEditFieldLabel.HorizontalAlignment = 'right';
            app.COMPortEditField = uieditfield(app.UIFigure, 'text');
            app.COMPortEditField.Position = [100 204 75 22];
            app.COMPortEditField.Value = 'COM3';

            % Baud Rate
            app.BaudRateEditFieldLabel = uilabel(app.UIFigure);
            app.BaudRateEditFieldLabel.Position = [185 204 80 22];
            app.BaudRateEditFieldLabel.Text = 'Baud Rate:';
            app.BaudRateEditFieldLabel.HorizontalAlignment = 'right';
            app.BaudRateEditField = uieditfield(app.UIFigure, 'numeric');
            app.BaudRateEditField.Position = [270 204 80 22];
            app.BaudRateEditField.Value = 9600;

            % Connect / Disconnect
            app.ConnectButton = uibutton(app.UIFigure, 'push');
            app.ConnectButton.ButtonPushedFcn = @(~,~) app.connectSerial();
            app.ConnectButton.Position = [14 172 100 26];
            app.ConnectButton.Text = 'Connect';

            app.DisconnectButton = uibutton(app.UIFigure, 'push');
            app.DisconnectButton.ButtonPushedFcn = @(~,~) app.disconnectSerial();
            app.DisconnectButton.Position = [122 172 110 26];
            app.DisconnectButton.Text = 'Disconnect';
            app.DisconnectButton.Enable = 'off';

            % Setpoint
            app.SetpointEditFieldLabel = uilabel(app.UIFigure);
            app.SetpointEditFieldLabel.Position = [14 132 110 22];
            app.SetpointEditFieldLabel.Text = 'Setpoint (5-45°C):';
            app.SetpointEditFieldLabel.HorizontalAlignment = 'right';
            app.SetpointEditField = uieditfield(app.UIFigure, 'numeric');
            app.SetpointEditField.Position = [130 132 70 22];
            app.SetpointEditField.Value = 25;

            app.SendSetpointButton = uibutton(app.UIFigure, 'push');
            app.SendSetpointButton.ButtonPushedFcn = @(~,~) app.sendSetpoint();
            app.SendSetpointButton.Position = [210 130 120 26];
            app.SendSetpointButton.Text = 'Send Setpoint';

            % Status
            app.StatusTextAreaLabel = uilabel(app.UIFigure);
            app.StatusTextAreaLabel.Position = [14 102 50 22];
            app.StatusTextAreaLabel.Text = 'Status:';
            app.StatusTextArea = uitextarea(app.UIFigure);
            app.StatusTextArea.Position = [14 14 360 84];
            app.StatusTextArea.Editable = 'off';

            % ── PID Sliders (BONUS) ──────────────────────────

            % Kp
            app.KpSliderLabel = uilabel(app.UIFigure);
            app.KpSliderLabel.Position = [400 204 25 22];
            app.KpSliderLabel.Text = 'Kp';
            app.KpSlider = uislider(app.UIFigure);
            app.KpSlider.Limits   = [0 100];
            app.KpSlider.Value    = 30;
            app.KpSlider.Position = [432 214 220 3];
            app.KpSlider.ValueChangedFcn = @(~,~) app.updateSliderLabels();
            app.KpValueLabel = uilabel(app.UIFigure);
            app.KpValueLabel.Position = [660 204 100 22];
            app.KpValueLabel.Text = 'Kp = 30.0';

            % Ki
            app.KiSliderLabel = uilabel(app.UIFigure);
            app.KiSliderLabel.Position = [400 155 25 22];
            app.KiSliderLabel.Text = 'Ki';
            app.KiSlider = uislider(app.UIFigure);
            app.KiSlider.Limits   = [0 1];
            app.KiSlider.Value    = 0.08;
            app.KiSlider.Position = [432 165 220 3];
            app.KiSlider.ValueChangedFcn = @(~,~) app.updateSliderLabels();
            app.KiValueLabel = uilabel(app.UIFigure);
            app.KiValueLabel.Position = [660 155 100 22];
            app.KiValueLabel.Text = 'Ki = 0.080';

            % Kd
            app.KdSliderLabel = uilabel(app.UIFigure);
            app.KdSliderLabel.Position = [400 106 25 22];
            app.KdSliderLabel.Text = 'Kd';
            app.KdSlider = uislider(app.UIFigure);
            app.KdSlider.Limits   = [0 20];
            app.KdSlider.Value    = 2;
            app.KdSlider.Position = [432 116 220 3];
            app.KdSlider.ValueChangedFcn = @(~,~) app.updateSliderLabels();
            app.KdValueLabel = uilabel(app.UIFigure);
            app.KdValueLabel.Position = [660 106 100 22];
            app.KdValueLabel.Text = 'Kd = 2.0';

            % Send PID button
            app.SendPIDButton = uibutton(app.UIFigure, 'push');
            app.SendPIDButton.ButtonPushedFcn = @(~,~) app.sendPID();
            app.SendPIDButton.Position = [432 55 220 30];
            app.SendPIDButton.Text = 'Send PID to MSP430';
            app.SendPIDButton.FontWeight = 'bold';

            app.UIFigure.Visible = 'on';
        end

    end

    % ── Constructor / destructor ─────────────────────────────
    methods (Access = public)

        function app = Project_appD_v2_final()
            createComponents(app);
            registerApp(app, app.UIFigure);

            % Startup defaults
            app.CurrentTempLabel.Text    = '-- °C';
            app.StatusTextArea.Value     = {'Ready. Enter COM port and press Connect.'};
            app.COMPortEditField.Value   = 'COM3';
            app.BaudRateEditField.Value  = 9600;
            app.SetpointEditField.Value  = 25;
            app.PWMLabel.Text            = 'PWM: --';
            app.updateSliderLabels();
            app.serialPort = [];
            app.timer_obj  = [];

            if nargout == 0
                clear app
            end
        end

        function delete(app)
            if ~isempty(app.timer_obj) && isvalid(app.timer_obj)
                stop(app.timer_obj);
                delete(app.timer_obj);
            end
            if ~isempty(app.serialPort)
                delete(app.serialPort);
            end
            if isvalid(app.UIFigure)
                delete(app.UIFigure);
            end
        end

    end
end
