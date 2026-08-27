function s = hilReadLine(h)
%SIMLAB.HILREADLINE  Read one response line, or '' on timeout.
    switch h.backend
        case 'serialport'
            try
                s = readline(h.obj);
            catch
                s = '';
            end
        otherwise
            s = fgetl(h.obj);
            if ~ischar(s)
                s = '';
            end
    end
    s = strtrim(s);
end
