 function [] = timerCallback(~,~,guiHandle)
 if ~isempty(guiHandle)
      % get the handles
      handles = guidata(guiHandle);
      if ~isempty(handles)
          a = fscanf(handles.arduino, '%f');
          
          while (a~= -10000)
              continue
          end
          % query your bluetooth board using your code from the while loop
          % if new data, then update the axes
           xdata = [get(handles.Pos,'XData') rand];
           ydata = [get(handles.Pos,'YData') rand];    
           set(handles.Pos,'XData',xdata,'YData',ydata);
           
           xdata = [get(handles.Hka,'XData') rand];
           ydata = [get(handles.Hka,'YData') rand];    
           set(handles.Hka,'XData',xdata,'YData',ydata);
           
           xdata = [get(handles.Hkt,'XData') rand];
           ydata = [get(handles.Hkt,'YData') rand];    
           set(handles.Hkt,'XData',xdata,'YData',ydata);
           
           xdata = [get(handles.Hha,'XData') rand];
           ydata = [get(handles.Hha,'YData') rand];    
           set(handles.Hha,'XData',xdata,'YData',ydata);
           
           xdata = [get(handles.Hht,'XData') rand];
           ydata = [get(handles.Hht,'YData') rand];    
           set(handles.Hht,'XData',xdata,'YData',ydata);
           
           set(handles.text4, 'String', num2str(rand));
           set(handles.text6, 'String', num2str(rand));
           set(handles.text7, 'String', num2str(rand));
           set(handles.text8, 'String', num2str(rand));
      end
  end