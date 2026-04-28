function f = pqa_freq(s)
	% sample freqency
	fs = 4194304/4/512;

	% remove DC offset from the input signal
	s = s - mean(s);

	% find zero crossing points
	[~, ~, s3] = zerocrossrate(s);
	zc = find(s3);

	% number of zero crossing points
	zcn = numel(zc);

	% calculate fractions for the first and last points
	zcf = abs(s(zc(2))) / (abs(s(zc(2))) + abs(s(zc(2) - 1)));
	zcl = abs(s(zc(end) - 1)) / (abs(s(zc(end))) + abs(s(zc(end) - 1)));

	% ignore the first point, it has been found as invalid
	f = 1 / (2 * (zc(end) - 1 - zc(2) + zcf + zcl) / (zcn - 2) / fs);
end