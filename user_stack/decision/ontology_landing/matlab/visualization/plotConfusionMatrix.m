function plotConfusionMatrix(metrics)
%PLOTCONFUSIONMATRIX Plot the confusion matrix.

figure('Name', 'Confusion Matrix');
imagesc(metrics.confusionMatrix); colorbar; axis equal tight;