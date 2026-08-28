function h = plotSensitivity(s, opt)
%SIMLAB.PLOTSENSITIVITY  The loop, S and T, on plain portable subplots.
%
%   SIMLAB.PLOTSENSITIVITY(S)        % S from simlab.sensitivity
%
% Five panels, no yyaxis and no plotyy. Both of those have died under real
% MATLAB in slightly different ways across releases (a deleted-Axes error in
% the plotyy path was reported from R2025b), and a diagnostic plot that
% cannot be drawn is worse than a plainer one. Two axes for the loop gain
% (magnitude and phase on their own scales) cost one extra row and nothing
% else.
%
%   1  |L| in dB, with the 0 dB line and the gain-crossover marked
%   2  phase of L in degrees, with -180 and the phase-crossover marked
%   3  |S| with the Ms peak and the 1.4 / 2.0 reference lines
%   4  |T| with the -3 dB bandwidth marked
%   5  the Nyquist plot - the distance to -1 at a glance, which is the
%      picture behind Ms

    if nargin < 2, opt = struct(); end
    fignum = 92;
    if isfield(opt, 'fig') && ~isempty(opt.fig), fignum = opt.fig; end

    h = figure(fignum);
    clf(h);
    set(h, 'Color', 'w', 'Name', 'PIDX loop analysis');

    w = s.w(:);
    magdB = 20 * log10(abs(s.L(:)));
    phdeg = 180 * unwrap(angle(s.L(:))) / pi;

    % ---- 1: loop gain magnitude ----
    ax1 = subplot(3, 2, 1, 'Parent', h);
    semilogx(w, magdB, 'b', 'LineWidth', 1.4);
    hold(ax1, 'on');
    semilogx(w, zeros(size(w)), 'k:', 'LineWidth', 0.8);
    if ~isnan(s.pmFreq)
        semilogx([s.pmFreq s.pmFreq], ylim(ax1), 'g--', 'LineWidth', 0.9);
    end
    hold(ax1, 'off');
    grid(ax1, 'on');
    ylabel(ax1, '|L| [dB]');
    title(ax1, sprintf('loop gain   GM %.2fx @ %.3g rad/s', s.gm, s.gmFreq));

    % ---- 2: loop phase ----
    ax2 = subplot(3, 2, 2, 'Parent', h);
    semilogx(w, phdeg, 'r', 'LineWidth', 1.2);
    hold(ax2, 'on');
    semilogx(w, -180 * ones(size(w)), 'r:', 'LineWidth', 0.8);
    if ~isnan(s.gmFreq)
        semilogx([s.gmFreq s.gmFreq], ylim(ax2), 'm--', 'LineWidth', 0.9);
    end
    hold(ax2, 'off');
    grid(ax2, 'on');
    ylabel(ax2, 'phase [deg]');
    title(ax2, sprintf('loop phase   PM %.1f deg @ %.3g rad/s', s.pm, s.pmFreq));

    % ---- 3: sensitivity ----
    ax3 = subplot(3, 2, 3, 'Parent', h);
    loglog(w, abs(s.S(:)), 'b', 'LineWidth', 1.4);
    hold(ax3, 'on');
    loglog(w, 1.4 * ones(size(w)), '--', 'Color', [0.2 0.7 0.2], 'LineWidth', 0.9);
    loglog(w, 2.0 * ones(size(w)), '--', 'Color', [0.9 0.3 0.2], 'LineWidth', 0.9);
    loglog(s.MsFreq, s.Ms, 'ko', 'MarkerFaceColor', 'k');
    hold(ax3, 'off');
    grid(ax3, 'on');
    ylabel(ax3, '|S|');
    legend(ax3, {'|S|', 'Ms = 1.4', 'Ms = 2.0'}, 'Location', 'best', 'FontSize', 7);
    title(ax3, sprintf('sensitivity   Ms = %.3f @ %.3g rad/s', s.Ms, s.MsFreq));

    % ---- 4: complementary sensitivity ----
    ax4 = subplot(3, 2, 4, 'Parent', h);
    loglog(w, abs(s.T(:)), 'Color', [0.6 0.2 0.6], 'LineWidth', 1.4);
    hold(ax4, 'on');
    if ~isnan(s.bandwidth)
        T0 = abs(s.T(1));
        loglog([s.bandwidth s.bandwidth], ylim(ax4), 'g--');
        loglog(w, (T0 / sqrt(2)) * ones(size(w)), 'k:', 'LineWidth', 0.7);
    end
    hold(ax4, 'off');
    grid(ax4, 'on');
    ylabel(ax4, '|T|');
    xlabel(ax4, '\omega [rad/s]');
    title(ax4, sprintf('complementary sensitivity   Mt = %.2f, bw %.3g rad/s', ...
        s.Mt, s.bandwidth));

    % ---- 5: Nyquist ----
    ax5 = subplot(3, 2, [5 6], 'Parent', h);
    % Clip near the origin pole: |L| explodes at low frequency and the
    % interesting part of the curve - its approach to -1 - would vanish off
    % the axes.
    lim = 4;
    Lc = s.L(:);
    big = abs(Lc) > lim;
    Lc(big) = lim * Lc(big) ./ abs(Lc(big));
    plot(real(Lc), imag(Lc), 'b', 'LineWidth', 1.2);
    hold(ax5, 'on');
    plot(real(conj(Lc)), imag(conj(Lc)), 'b', 'LineWidth', 0.5);
    th = linspace(0, 2 * pi, 200);
    plot(cos(th), sin(th), 'k:', 'LineWidth', 0.7);
    plot(-1, 0, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    axis(ax5, 'equal');
    xlim(ax5, [-lim lim]);
    ylim(ax5, [-lim lim]);
    hold(ax5, 'off');
    grid(ax5, 'on');
    xlabel(ax5, 'Re');
    ylabel(ax5, 'Im');
    title(ax5, 'Nyquist: distance to -1 is 1/Ms');

    try
        sgtitle(s.verdict, 'Interpreter', 'none');
    catch
        set(h, 'Name', s.verdict);
    end
end
