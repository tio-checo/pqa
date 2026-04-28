classdef pqa_dashboard_exported < matlab.apps.AppBase

	% Properties that correspond to app components
	properties (Access = public)
		UIFigure           matlab.ui.Figure
		GridLayout         matlab.ui.container.GridLayout
		I341AGauge         matlab.ui.control.Gauge
		I341AGaugeLabel    matlab.ui.control.Label
		I241AGauge         matlab.ui.control.Gauge
		I241AGaugeLabel    matlab.ui.control.Label
		U32402VGauge       matlab.ui.control.Gauge
		U32402VGaugeLabel  matlab.ui.control.Label
		U22402VGauge       matlab.ui.control.Gauge
		U22402VGaugeLabel  matlab.ui.control.Label
		I141AGauge         matlab.ui.control.Gauge
		I141AGaugeLabel    matlab.ui.control.Label
		U12402VGauge       matlab.ui.control.Gauge
		U12402VLabel       matlab.ui.control.Label
	end

	% Component initialization
	methods (Access = private)

		% Create UIFigure and components
		function createComponents(app)

			% Create UIFigure and hide until all components are created
			app.UIFigure = uifigure('Visible', 'off');
			app.UIFigure.Position = [100 100 789 987];
			app.UIFigure.Name = 'MATLAB App';

			% Create GridLayout
			app.GridLayout = uigridlayout(app.UIFigure);
			app.GridLayout.ColumnWidth = {'1x', '1x', '1x'};
			app.GridLayout.RowHeight = {'1x', '0.2x', '1x', '0.2x', '0.5x', '2x'};

			% Create U12402VLabel
			app.U12402VLabel = uilabel(app.GridLayout);
			app.U12402VLabel.HorizontalAlignment = 'center';
			app.U12402VLabel.VerticalAlignment = 'top';
			app.U12402VLabel.FontSize = 18;
			app.U12402VLabel.FontWeight = 'bold';
			app.U12402VLabel.Layout.Row = 2;
			app.U12402VLabel.Layout.Column = 1;
			app.U12402VLabel.Text = 'U1 240.2V';

			% Create U12402VGauge
			app.U12402VGauge = uigauge(app.GridLayout, 'circular');
			app.U12402VGauge.Limits = [200 260];
			app.U12402VGauge.MajorTicks = [200 205 210 215 220 225 230 235 240 245 250 255 260];
			app.U12402VGauge.FontSize = 11;
			app.U12402VGauge.Layout.Row = 1;
			app.U12402VGauge.Layout.Column = 1;
			app.U12402VGauge.Value = 240.2;

			% Create I141AGaugeLabel
			app.I141AGaugeLabel = uilabel(app.GridLayout);
			app.I141AGaugeLabel.HorizontalAlignment = 'center';
			app.I141AGaugeLabel.VerticalAlignment = 'top';
			app.I141AGaugeLabel.FontSize = 18;
			app.I141AGaugeLabel.FontWeight = 'bold';
			app.I141AGaugeLabel.Layout.Row = 4;
			app.I141AGaugeLabel.Layout.Column = 1;
			app.I141AGaugeLabel.Text = 'I1 4.1A';

			% Create I141AGauge
			app.I141AGauge = uigauge(app.GridLayout, 'circular');
			app.I141AGauge.Limits = [0 10];
			app.I141AGauge.FontSize = 11;
			app.I141AGauge.Layout.Row = 3;
			app.I141AGauge.Layout.Column = 1;
			app.I141AGauge.Value = 4.1;

			% Create U22402VGaugeLabel
			app.U22402VGaugeLabel = uilabel(app.GridLayout);
			app.U22402VGaugeLabel.HorizontalAlignment = 'center';
			app.U22402VGaugeLabel.VerticalAlignment = 'top';
			app.U22402VGaugeLabel.FontSize = 18;
			app.U22402VGaugeLabel.FontWeight = 'bold';
			app.U22402VGaugeLabel.Layout.Row = 2;
			app.U22402VGaugeLabel.Layout.Column = 2;
			app.U22402VGaugeLabel.Text = 'U2 240.2V';

			% Create U22402VGauge
			app.U22402VGauge = uigauge(app.GridLayout, 'circular');
			app.U22402VGauge.Limits = [200 260];
			app.U22402VGauge.MajorTicks = [200 205 210 215 220 225 230 235 240 245 250 255 260];
			app.U22402VGauge.FontSize = 11;
			app.U22402VGauge.Layout.Row = 1;
			app.U22402VGauge.Layout.Column = 2;
			app.U22402VGauge.Value = 240.2;

			% Create U32402VGaugeLabel
			app.U32402VGaugeLabel = uilabel(app.GridLayout);
			app.U32402VGaugeLabel.HorizontalAlignment = 'center';
			app.U32402VGaugeLabel.VerticalAlignment = 'top';
			app.U32402VGaugeLabel.FontSize = 18;
			app.U32402VGaugeLabel.FontWeight = 'bold';
			app.U32402VGaugeLabel.Layout.Row = 2;
			app.U32402VGaugeLabel.Layout.Column = 3;
			app.U32402VGaugeLabel.Text = 'U3 240.2V';

			% Create U32402VGauge
			app.U32402VGauge = uigauge(app.GridLayout, 'circular');
			app.U32402VGauge.Limits = [200 260];
			app.U32402VGauge.MajorTicks = [200 205 210 215 220 225 230 235 240 245 250 255 260];
			app.U32402VGauge.FontSize = 11;
			app.U32402VGauge.Layout.Row = 1;
			app.U32402VGauge.Layout.Column = 3;
			app.U32402VGauge.Value = 240.2;

			% Create I241AGaugeLabel
			app.I241AGaugeLabel = uilabel(app.GridLayout);
			app.I241AGaugeLabel.HorizontalAlignment = 'center';
			app.I241AGaugeLabel.VerticalAlignment = 'top';
			app.I241AGaugeLabel.FontSize = 18;
			app.I241AGaugeLabel.FontWeight = 'bold';
			app.I241AGaugeLabel.Layout.Row = 4;
			app.I241AGaugeLabel.Layout.Column = 2;
			app.I241AGaugeLabel.Text = 'I2 4.1A';

			% Create I241AGauge
			app.I241AGauge = uigauge(app.GridLayout, 'circular');
			app.I241AGauge.Limits = [0 10];
			app.I241AGauge.FontSize = 11;
			app.I241AGauge.Layout.Row = 3;
			app.I241AGauge.Layout.Column = 2;
			app.I241AGauge.Value = 4.1;

			% Create I341AGaugeLabel
			app.I341AGaugeLabel = uilabel(app.GridLayout);
			app.I341AGaugeLabel.HorizontalAlignment = 'center';
			app.I341AGaugeLabel.VerticalAlignment = 'top';
			app.I341AGaugeLabel.FontSize = 18;
			app.I341AGaugeLabel.FontWeight = 'bold';
			app.I341AGaugeLabel.Layout.Row = 4;
			app.I341AGaugeLabel.Layout.Column = 3;
			app.I341AGaugeLabel.Text = 'I3 4.1A';

			% Create I341AGauge
			app.I341AGauge = uigauge(app.GridLayout, 'circular');
			app.I341AGauge.Limits = [0 10];
			app.I341AGauge.FontSize = 11;
			app.I341AGauge.Layout.Row = 3;
			app.I341AGauge.Layout.Column = 3;
			app.I341AGauge.Value = 4.1;

			% Show the figure after all components are created
			app.UIFigure.Visible = 'on';
		end
	end

	% App creation and deletion
	methods (Access = public)

		% Construct app
		function app = pqa_dashboard_exported

			% Create UIFigure and components
			createComponents(app)

			% Register the app with App Designer
			registerApp(app, app.UIFigure)

			if nargout == 0
				clear app
			end
		end

		% Code that executes before app deletion
		function delete(app)

			% Delete UIFigure when app is deleted
			delete(app.UIFigure)
		end
	end
end