function hilFlush(h)
%SIMLAB.HILFLUSH  Discard anything the board sent that nobody asked for.
    switch h.backend
        case 'serialport'
            flush(h.obj, 'input');
        otherwise
            try
                while h.obj.BytesAvailable > 0
                    fread(h.obj, h.obj.BytesAvailable);
                end
            catch
                % older releases without BytesAvailable: nothing to flush
            end
    end
end
