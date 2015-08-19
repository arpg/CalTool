function R = makeBlkDiag(src, count)
    hund = [];      
    if (count > 100)

        for ii=1:100
            hund = blkdiag(hund, src);
        end
    end
    
    hundCount = floor(count / 100);
    total = [];
    for ii=1:hundCount
        total = blkdiag(total, hund);
    end
    
    for ii=1:(count - 100*hundCount)
        total = blkdiag(total, src);
    end
    size(total);
    R = total;
end
