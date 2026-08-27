function h = plotSensitivity(s, opt)
%SIMLAB.PLOTSENSITIVITY  Bode-style plot of the loop, S and T.
%
%   SIMLAB.PLOTSENSITIVITY(S)        % S from simlab.sensitivity
%
% Four panels:
%   1  |L| and the phase of L, with the -180 deg and 0 dB lines, and the
%      gain- and phase-margin crossings marked
%   2  |S| with the Ms peak and the 1.4 / 2.0 reference lines
%   3  |T| with the -3 dB bandwidth marked
%   4  the Nyquist plot, which shows the distance to -1 at a glance - the
%      picture behind Ms
%
% Plain loglog/semilogx rather than bode(): no Control System Toolbox, and
% the curves are the ones simlab.sensitivity actually computed, so the plot
% and the numbers cannot disagree.

    if nargin < 2, opt = struct(); end
    fignum = 92;
    if isfield(opt, 'fig') && ~isempty(opt.fig), fignum = opt.fig; end

    h = figure(fignum);
    clf(h);
    set(h, 'Color', 'w', 'Name', 'PIDX loop analysis');

    w = s.w(:);

    % ---- 1: loop gain ----
    % Two vertical scales: magnitude and phase. yyaxis is R2016a+ and absent
    % from older Octave, so fall back to plotyy rather than dropping the
    % phase trace - the margins live in the phase.
    mag = 20 * log10(abs(s.L(:)));
    ph = 180 * unwrap(angle(s.L(:))) / pi;
    ax1 = subplot(2, 2, 1, 'Parent', h);
    haveYY = (exist('yyaxis', 'file') == 2) || (exist('yyaxis', 'builtin') == 5);
    if haveYY
        yyaxis(ax1, 'left');
        semilogx(w, mag, 'b', 'LineWidth', 1.4);
        ylabel(ax1, '|L| [dB]');
        hold(ax1, 'on');
        semilogx(w, zeros(size(w)), 'k:', 'LineWidth', 0.8);
        yyaxis(ax1, 'right');
        semilogx(w, ph, 'r', 'LineWidth', 1.2);
        ylabel(ax1, 'phase [deg]');
        semilogx(w, -180 * ones(size(w)), 'r:', 'LineWidth', 0.8);
        if ~isnan(s.pmFreq)
            semilogx([s.pmFreq s.pmFreq], [-180 0], 'g--', 'LineWidth', 0.9);
        end
        if ~isnan(s.gmFreq)
            semilogx([s.gmFreq s.gmFreq], [-180 0], 'm--', 'LineWidth', 0.9);
        end
        hold(ax1, 'off');
    else
        axb = plotyy(w, mag, w, ph, @semilogx, @semilogx);
        set(get(axb(1), 'Children'), 'LineWidth', 1.4);
        set(get(axb(2), 'Children'), 'LineWidth', 1.2);
        ylabel(axb(1), '|L| [dB]');
        ylabel(axb(2), 'phase [deg]');
        hold(axb(2), 'on');
        semilogx(w, -180 * ones(size(w)), 'r:', 'LineWidth', 0.8);
        hold(axb(2), 'off');
        ax1 = axb(1);
    end
    grid(ax1, 'on');
    xlabel(ax1, '\omega [rad/s]');

    title(ax1, sprintf('loop L(j\\omega)   GM %.2fx @ %.3g rad/s, PM %.1f deg @ %.3g rad/s', ...
        s.gm, s.gmFreq, s.pm, s.pmFreq));

    % ---- 2: sensitivity ----
    ax2 = subplot(2, 2, 2, 'Parent', h);
    loglog(w, abs(s.S(:)), 'b', 'LineWidth', 1.4);
    hold(ax2, 'on');
    loglog(w, 1.4 * ones(size(w)), '--', 'Color', [0.2 0.7 0.2], 'LineWidth', 0.9);
    loglog(w, 2.0 * ones(size(w)), '--', 'Color', [0.9 0.3 0.2], 'LineWidth', 0.9);
    loglog([w(1) w(end)], [1 1], 'k:', 'LineWidth', 0.7);
    loglog(s.MsFreq, s.Ms, 'ko', 'MarkerFaceColor', 'k');
    hold(ax2, 'off');
    grid(ax2, 'on');
    xlabel(ax2, '\omega [rad/s]');
    ylabel(ax2, '|S|');
    legend(ax2, {'|S|', 'Ms = 1.4', 'Ms = 2.0'}, 'Location', 'best', 'FontSize', 7);
    title(ax2, sprintf('sensitivity   Ms = %.3f @ %.3g rad/s', s.Ms, s.MsFreq));

    % ---- 3: complementary sensitivity ----
    ax3 = subplot(2, 2, 3, 'Parent', h);
    loglog(w, abs(s.T(:)), 'Color', [0.6 0.2 0.6], 'LineWidth', 1.4);
    hold(ax3, 'on');
    if ~isnan(s.bandwidth)
        T0 = abs(s.T(1));
        loglog([s.bandwidth s.bandwidth], [1e-3 T0 / sqrt(2)], 'g--');
        loglog(w, (T0 / sqrt(2)) * ones(size(w)), 'k:', 'LineWidth', 0.7);
    end
    hold(ax3, 'off');
    grid(ax3, 'on');
    xlabel(ax3, '\omega [rad/s]');
    ylabel(ax3, '|T|');
    title(ax3, sprintf('complementary sensitivity   Mt = %.2f, bandwidth %.3g rad/s', ...
        s.Mt, s.bandwidth));

    % ---- 4: Nyquist ----
    ax4 = subplot(2, 2, 4, 'Parent', h);
    % Clip the plot: near an integrator |L| explodes and the interesting part
    % of the curve - its approach to -1 - disappears off the axes.
    lim = 4;
    Lc = s.L(:);
    Lc(abs(Lc) > lim) = lim * Lc(abs(Lc) > lim) ./ abs(Lc(abs(Lc) > lim));
    plot(real(Lc), imag(Lc), 'b', 'LineWidth', 1.2);
    hold(ax4, 'on');
    plot(real(conj(Lc)), imag(conj(Lc)), 'b', 'LineWidth', 0.5);
    th = linspace(0, 2 * pi, 200);
    plot(cos(th), sin(th), 'k:', 'LineWidth', 0.7);
    plot(-1, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    axis(ax4, 'equal');
    xlim(ax4, [-lim lim]); ylim(ax4, [-lim lim]);
    hold(ax4, 'off');
    grid(ax4, 'on');
    xlabel(ax4, 'Re');
    ylabel(ax4, 'Im');
    title(ax4, 'Nyquist: distance to -1 is 1/Ms');

    sgtitleOrTitle(h, s.verdict);
end

function sgtitleOrTitle(h, ttl)
    try
        sgtitle(ttl, 'Interpreter', 'none');
    catch
        set(h, 'Name', ttl);
    end
end
