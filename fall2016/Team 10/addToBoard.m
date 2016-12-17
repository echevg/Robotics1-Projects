function [new_board, new_occfn] = addToBoard(s,block,board,occfn)
% block options: 'left', 'straight', 'right'
% current_direction: 1-up 2-left 3-down 4-right

new_board = board;
new_occfn = occfn;

[row,col] = find(occfn<0);
direction_val = -occfn(row,col);

if strcmp(block,'left')
    b = makeBlock(1,1,s);
    piece_val = 1;
    new_direction_val = mod(direction_val,4)+1;
elseif strcmp(block,'right')
    b = makeBlock(1,2,s);
    piece_val = 2;
    new_direction_val = mod(direction_val-2,4)+1;
else
    b = makeBlock(0,0,s);
    piece_val = 0;
    new_direction_val = direction_val;
end

if size(new_board,1) + size(new_board,2) > 0
    if size(occfn,1)*s > size(new_board,1)
        new_board = [zeros(s*(row==1),size(new_board,2));...
            new_board;...
            zeros(((row-1)*s==size(new_board,1))*s,size(new_board,2))];
    elseif size(occfn,2)*s > size(new_board,2)
        new_board = [zeros(size(new_board,1),(col==1)*s)...
            new_board...
            (zeros(size(new_board,1),(col-1)*s==size(new_board,2))*s)];
    end
end

new_board((row-1)*s+(1:s),(col-1)*s+(1:s)) = imrotate(b,90*(direction_val-1));
new_occfn(row,col) = direction_val-1 + piece_val*4;

if new_direction_val == 1
    if row == 1
        new_occfn = [nan(1,size(new_occfn,2)); new_occfn];
    else
        row = row - 1;
    end
elseif new_direction_val == 2
    if col == 1
        new_occfn = [nan(size(new_occfn,1),1) new_occfn];
    else
        col = col - 1;
    end
elseif new_direction_val == 3
    if row == size(new_occfn,1)
        new_occfn = [new_occfn; nan(1,size(new_occfn,2))];
    end
    row = row + 1;
else
    if col == size(new_occfn,2)
        new_occfn = [new_occfn nan(size(new_occfn,1))];
    end
    col = col + 1;
end

if isnan(new_occfn(row,col))
    new_occfn(row,col) = -new_direction_val;
else
    fprintf('Circuit is complete!\n')
end

end