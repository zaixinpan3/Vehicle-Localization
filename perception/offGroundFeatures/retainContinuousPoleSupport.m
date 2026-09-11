function retained = retainContinuousPoleSupport(z, cfg)
% retainContinuousPoleSupport: Reject vertically disconnected pole fragments.
% Use actual metric spacing of accepted returns, independent of scan order.
% Each connected run must itself satisfy the pole height and point minimums.
    retained=false(size(z));
    if isempty(z),return;end
    [ordered,order]=sort(z(:));
    group=cumsum([1;diff(ordered)>cfg.poleMaximumVerticalGapMeters]);
    count=accumarray(group,1);
    span=accumarray(group,ordered,[],@max)-accumarray(group,ordered,[],@min);
    valid=count>=cfg.poleMinimumPoints & span>=cfg.poleMinimumHeight;
    retained(order)=valid(group);
end
