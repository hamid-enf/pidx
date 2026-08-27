function h = hilConnect(opt)
%SIMLAB.HILCONNECT  Open the link to a board running the HIL firmware.
%
%   H = SIMLAB.HILCONNECT()                       % COM3 / /dev/ttyUSB0, 115200
%   H = SIMLAB.HILCONNECT('port', 'COM5', 'baud', 921600, 'timeout', 2)
%
% Returns a handle struct that simlab.hilRun drives. Nothing is imported from
% the Instrument Control Toolbox at file scope, so this file LOADS on a
% machine with no toolbox - it fails only when you actually try to connect,
% and then it says exactly what is missing.
%
% THE FIRMWARE
%   tools/hil/hil_board.c. It includes the file that simlab.exportSTM32 wrote
%   for your loop, so the controller running on the board is the controller
%   you designed here - the same translation unit, not a reimplementation.
%
%       cd tools/hil
%       make TUNING=/path/to/pidx_tuning_myLoop.h SYMBOL=myLoop
%       # flash hil_board.elf, then:
%   h = simlab.hilConnect('port', 'COM5');
%   r = simlab.hilRun(h, plant, cfg, scenario);
%
% WHAT THE LOOP ACTUALLY IS
%   The plant runs in MATLAB; the controller runs on the board. Each sample is
%   one round trip: MATLAB sends the measurement, the board runs PID_Update
%   and sends the command back. So the "plant" can be a model, a
%   second board, or real hardware wired to a DAC - from this side it is the
%   same call.
%
%   The consequence is that the sample rate is bounded by the round trip, not
%   by the board. At 115200 baud a round trip is a couple of milliseconds, so
%   this suits process loops at 10-500 Hz. It does not suit a 20 kHz current
%   loop, and no amount of buffering changes that - a controller whose dt
%   depends on the host is not the controller you are testing.
%
% OPTIONS
%   'port'      default COM3 on Windows, /dev/ttyUSB0 elsewhere
%   'baud'      default 115200
%   'timeout'   read timeout [s], default 1
%   'quiet'     do not print the handshake

    if nargin < 1, opt = struct(); end
    o = fillOpt(opt, 'port', defaultPort());
    o = fillOpt(opt, 'baud', 115200);
    o = fillOpt(opt, 'timeout', 1.0);
    o = fillOpt(opt, 'quiet', false);

    h = struct();
    h.port = o.port;
    h.baud = o.baud;
    h.timeout = o.timeout;
    h.obj = [];
    h.backend = '';

    % ---- serialport (R2019b+, base MATLAB, no toolbox) ----
    if exist('serialport', 'file') == 2
        try
            h.obj = serialport(o.port, o.baud, 'Timeout', o.timeout);
            h.backend = 'serialport';
        catch err
            h.obj = [];
            if ~o.quiet
                fprintf('  serialport failed: %s\n', err.message);
            end
        end
    end

    % ---- serial (Instrument Control Toolbox, older releases) ----
    if isempty(h.obj) && exist('serial', 'file') == 2
        try
            s = serial(o.port, 'BaudRate', o.baud, 'Timeout', o.timeout);
            fopen(s);
            h.obj = s;
            h.backend = 'serial';
        catch err
            h.obj = [];
            if ~o.quiet
                fprintf('  serial failed: %s\n', err.message);
            end
        end
    end

    if isempty(h.obj)
        error('simlab:hilConnect:noLink', ...
              ['could not open %s at %d baud. This needs either ' ...
               'serialport() (R2019b+, no toolbox) or the Instrument Control ' ...
               'Toolbox serial(). Check the port name and that nothing else ' ...
               'has it open.'], o.port, o.baud);
    end

    % ---- handshake ----
    %
    % Asking the board what it is running is not a formality. The most likely
    % failure of a HIL session is talking to a board that has the PREVIOUS
    % tuning flashed, and every number that follows would then be a
    % measurement of a controller you did not design.
    hilFlush(h);
    hilWrite(h, 'ID');
    id = hilReadLine(h);
    h.boardId = id;
    if ~o.quiet
        fprintf('simlab.hilConnect: %s at %d baud\n', o.port, o.baud);
        fprintf('  board: %s\n', id);
    end
    if isempty(id)
        warning('simlab:hilConnect:noId', ...
                ['the board did not answer ID. Either it is not running ' ...
                 'tools/hil/hil_board.c, or the wiring/baud is wrong. ' ...
                 'Everything after this point is unreliable.']);
    end
end

% ---------------------------------------------------------------------------
