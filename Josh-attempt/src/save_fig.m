function save_fig(fig, out_dir, file_stem)
%SAVE_FIG Save figure to PNG and FIG.

if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end
% Hide figure/axes toolbars before export to avoid toolbar artifacts.
set(findall(fig, 'Type', 'figure'), 'ToolBar', 'none');
ax = findall(fig, 'Type', 'axes');
for k = 1:numel(ax)
    if isprop(ax(k), 'Toolbar')
        ax(k).Toolbar.Visible = 'off';
    end
end

exportgraphics(fig, fullfile(out_dir, [file_stem '.png']), 'Resolution', 150);
savefig(fig, fullfile(out_dir, [file_stem '.fig']));

end
