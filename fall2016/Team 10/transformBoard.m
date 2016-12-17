function out = transformBoard(board,campos,theta)

out = [zeros(size(board,1),size(board,2)-2*campos(2)), board zeros(size(board,1), 2*campos(2)-size(board,2))];
out = [zeros(size(out,1)-2*campos(1),size(out,2));
    out;
    zeros(2*campos(1)-size(out,1),size(out,2))];

out = imrotate(out,-theta);

end