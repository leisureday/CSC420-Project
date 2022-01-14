function [xs,ys] =  getcube2D(vs)
xs = [vs(1,[1 2 4 3 1]);
      vs(1,[5 6 8 7 5]);
      vs(1,[1 5 7 3 1]);
      vs(1,[2 6 8 4 2])]';
ys = [vs(2,[1 2 4 3 1]);
      vs(2,[5 6 8 7 5]);
      vs(2,[1 5 7 3 1]);
      vs(2,[2 6 8 4 2])]';
end