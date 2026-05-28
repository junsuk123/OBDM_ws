function comparison = compareWithBaseline(aiResults, baselineResults, cfg)
%COMPAREWITHBASELINE Compare AI method against the threshold baseline.

comparison = struct();
comparison.ai = aiResults;
comparison.baseline = baselineResults;
comparison.improvement.accuracy = aiResults.accuracy - baselineResults.accuracy;
comparison.improvement.f1Score = aiResults.f1Score - baselineResults.f1Score;
comparison.improvement.unsafeLandingAttemptRate = baselineResults.unsafeLandingAttemptRate - aiResults.unsafeLandingAttemptRate;