function hilWrite(h, line)
%SIMLAB.HILWRITE  Send one command line to the board.
    switch h.backend
        case 'serialport'
            write(h.obj, [line, sprintf('\n')], 'string');
        otherwise
            fprintf(h.obj, '%s\n', line);
    end
end
