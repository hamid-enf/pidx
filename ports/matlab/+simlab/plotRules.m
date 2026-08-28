function h = plotRules(c, opt)
%SIMLAB.PLOTRULES  The two rankings, side by side.
%
%   SIMLAB.PLOTRULES(C)         % C from simlab.compareRules
%
% The picture behind the finding in README.md. Panel 1 ranks the rules by IAE
% on the model; panel 2 ranks them by survival when the plant is wrong. If
% the two pictures were the same, tuning would be a solved problem. They are
% close to opposite, and this figure is how you see it on YOUR plant rather
% than taking the C study's word for it.
%
% Panel 3 is the payoff: IAE against survival for every rule, so you can see
% which corner of the trade-off you are buying. The rules in the upper left
% (fast but fragile) are the ones the datasheets show.

    if nargin < 2, opt = struct(); end
    fignum = 93;
    if isfield(opt, 'fig') && ~isempty(opt.fig), fignum = opt.fig; end

    h = figure(fignum);
    clf(h);
    set(h, 'Color', 'w', 'Name', 'PIDX tuning-rule comparison');

    t = c.table;
    ok = find(t.ok & ~isnan(t.iae));
    names = t.name(ok);
    iae = t.iae(ok);
    surv = 100 * t.survival(ok);

    % ---- 1: ranked by IAE ----
    ax1 = subplot(1, 3, 1, 'Parent', h);
    [~, ord] = sort(iae);
    barh(ax1, numel(ok):-1:1, iae(ord), 'FaceColor', [0.3 0.5 0.8]);
    set(ax1, 'YTick', 1:numel(ok), 'YTickLabel', flipud(names(ord)), ...
        'FontSize', 7);
    xlabel(ax1, 'IAE (lower is faster)');
    title(ax1, sprintf('ranked by IAE  [%s mode]', c.mode));
    grid(ax1, 'on');

    % ---- 2: ranked by survival ----
    ax2 = subplot(1, 3, 2, 'Parent', h);
    [~, ord2] = sort(-surv);
    cols = repmat([0.3 0.7 0.4], numel(ok), 1);
    cols(surv(ord2) < 90, :) = repmat([0.95 0.75 0.2], sum(surv(ord2) < 90), 1);
    cols(surv(ord2) < 75, :) = repmat([0.9 0.3 0.25], sum(surv(ord2) < 75), 1);
    barh(ax2, numel(ok):-1:1, surv(ord2));
    ch = get(ax2, 'Children');
    if ~isempty(ch)
        set(ch(1), 'FaceColor', 'flat', 'CData', cols);
    end
    set(ax2, 'YTick', 1:numel(ok), 'YTickLabel', flipud(names(ord2)), ...
        'FontSize', 7);
    xlim(ax2, [0 100]);
    xlabel(ax2, 'survival [%]');
    title(ax2, sprintf('ranked by survival over %d wrong plants', c.nRuns));
    grid(ax2, 'on');

    % ---- 3: the trade-off ----
    ax3 = subplot(1, 3, 3, 'Parent', h);
    plot(ax3, iae, surv, 'o', 'MarkerSize', 7, ...
         'MarkerFaceColor', [0.3 0.5 0.8], 'MarkerEdgeColor', 'k');
    hold(ax3, 'on');
    for i = 1:numel(ok)
        text(ax3, iae(i), surv(i), sprintf('  %s', names{i}), 'FontSize', 7);
    end
    if ~isnan(c.spearman)
        % A straight line through the cloud makes the correlation visible
        % without the reader having to do it in their head.
        pp = polyfit(iae, surv, 1);
        xr = [min(iae), max(iae)];
        plot(ax3, xr, polyval(pp, xr), 'k--', 'LineWidth', 0.9);
    end
    hold(ax3, 'off');
    xlabel(ax3, 'IAE (speed on a perfect model)');
    ylabel(ax3, 'survival [%] (robustness)');
    grid(ax3, 'on');
    title(ax3, sprintf('the trade-off   Spearman \\rho = %.3f', c.spearman));

    if ~isempty(c.best)
        sgtitleOrTitle(h, sprintf( ...
            'tuning rules on this plant  -  recommended: %s (%s)', ...
            c.best.name, c.recommend));
    else
        sgtitleOrTitle(h, 'tuning rules on this plant');
    end
end

function sgtitleOrTitle(h, ttl)
    try
        sgtitle(ttl, 'Interpreter', 'none');
    catch
        set(h, 'Name', ttl);
    end
end
