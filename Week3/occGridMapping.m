% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
myResol = param.resol;
% % the initial map size in pixels
myMap = zeros(param.size);
% % the origin of the map in pixels
myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
figure, 
h = imagesc(myMap);
colormap('gray'); axis equal;
hold on;

for j = 1:N % for each time,

      
    % Find grids hit by the rays (in the gird map coordinate)

	realLoc = [ranges(:, j) .* cos(scanAngles + pose(3, j)), -ranges(:,j) .* sin(scanAngles + pose(3, j))]' + repmat(pose(1:2, j), [1 numel(scanAngles)]);
  	occLoc = ceil(realLoc .* myResol) + repmat(myorigin, [1 numel(scanAngles)]);  	
    occLoc = unique(occLoc', 'rows')'; % find unique occ Location to reduce computation load
    
    % Find occupied-measurement cells and free-measurement cells
	poseLoc = ceil(pose(1:2, j) .* myResol) + myorigin;
	freeLoc = [];
	for k = 1:size(occLoc, 2)
		[freex, freey] = bresenham(poseLoc(1), poseLoc(2), occLoc(1, k), occLoc(2, k));  
		free = sub2ind(size(myMap),freey,freex);
		freeLoc = [freeLoc; (free)];
	end

    % Update the log-odds
    % occLoc = sub2ind(size(myMap), occLoc(2, :), occLoc(1, :));
	myMap(occLoc) = myMap(occLoc) + lo_occ;
	myMap(freeLoc) = myMap(freeLoc) - lo_free;

    % Saturate the log-odd values
	myMap(myMap > lo_max) = lo_max;
	myMap(myMap < lo_min) = lo_min;

    % Visualize the map as needed
    h.CData = myMap;
    plot(poseLoc(1), poseLoc(2), 'r.');
    drawnow

end

end

