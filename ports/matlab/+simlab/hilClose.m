function hilClose(h)
%SIMLAB.HILCLOSE  Close the link. Safe to call twice.
    try
        switch h.backend
            case 'serialport'
                delete(h.obj);
            otherwise
                fclose(h.obj);
                delete(h.obj);
        end
    catch
        % already closed
    end
end

function p = defaultPort()
    if ispc()
        p = 'COM3';
    else
        p = '/dev/ttyUSB0';
    end
end

function o = fillOpt(o, name, default)
    if ~isfield(o, name) || isempty(o.(name))
        o.(name) = default;
    end
end
