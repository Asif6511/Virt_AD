%% Tennis Game

close all
clear all
%% Reading the video

fname = 'C:\UT Files\Image Processing and Computer Vision\Final Project\tennis.mp4';
vidReader = VideoReader(fname);

%% Canny Edge Detection & Writing the video

       edgeplay=VideoWriter('cannyedge_tennis.avi');
       open(edgeplay);
while hasFrame(vidReader)
    frameRGB = readFrame(vidReader);
    frameGray = rgb2gray(frameRGB);
     edge1=edge(frameGray,'Canny',[0.25 0.55]);
          bw = colormap(gray(2));
          frame = im2frame(uint8(edge1), bw);
          writeVideo(edgeplay,frame);
end
         close(edgeplay);

%% Feature Points Estimation across all present frames

gname='C:\UT Files\Image Processing and Computer Vision\Final Project\cannyedge_tennis.avi';
vidReader1=VideoReader(gname);
count =0;
nFrames = vidReader1.NumberOfFrames;
for iFrame=1:1:nFrames
    frame1=read(vidReader1,iFrame);
    frameGray1 = rgb2gray(frame1);
    count = count+1;
    [accum, theta, rho]=hough(frameGray1);
    peaks=houghpeaks(accum,1000,'Threshold',ceil(0.35*max(accum(:))));
    line_seg=houghlines(frameGray1,theta,rho,peaks,'FillGap',600,'MinLength',600);
    seg=[];
     imshow(frame1);hold on
    for k=1:length(line_seg) % Filtering the spurious line segments
        if ((((line_seg(k).rho <-800 && line_seg(k).rho >-950)|| line_seg(k).rho >-300)&& line_seg(k).theta==-90)||(line_seg(k).rho>0 && (line_seg(k).theta<-90 || line_seg(k).theta<80))) 
        seg=cat(1,seg,line_seg(k));
        endpoint=[line_seg(k).point1; line_seg(k).point2];
        end
    end
    f=[];
    g=[];
    for i=1:length(seg) % Finding the Point of Intersection of each lines
        for j=i:length(seg)
        [a]=seg(i).point1;
        [b]=seg(i).point2;
        [c]=seg(j).point1;
        [d]=seg(j).point2;
        res=((a(2)-b(2))*(c(1)-d(1)))-((a(1)-b(1))*(c(2)-d(2)));
        if (res~=0)
        [e]=[(((a(2)*b(1)-a(1)*b(2))*(c(2)-d(2))-(a(2)-b(2))*(c(2)*d(1)-c(1)*d(2)))/res),(((a(2)*b(1)-a(1)*b(2))*(c(1)-d(1))-(a(1)-b(1))*(c(2)*d(1)-c(1)*d(2)))/res)];
       if (e(1)>0)
        f=cat(1,f,e);
       end
        end
        end
    end
    f=fliplr(f); % Maintaining the same feature point order across all frames in (x,y) indexs
     fi=f(1:end/2,:);
        fi=sort(fi);
        fi1=f(end/2 +1:end,:);
        fi1=sort(fi1);
        g=cat(1,fi,fi1);
        g=g';
        g=sortrows(g,2);
        g=g';
        % Changing the index and point order of certain frames found by
        % peruse
        if (iFrame==123 || iFrame==124 || iFrame==125 || iFrame==128 || iFrame==131 || iFrame==133 || iFrame==153 || iFrame==154 || iFrame==224 || iFrame==225 || iFrame==226 || iFrame==227 || iFrame==228)
            g1=g(1:end/2,:);
            g1=fliplr(g1);
            g2=g(end/2 +1:end,:);
            g2=fliplr(g2);
            g=cat(1,g2,g1);
        end       
     % Drawing a rectangle on the point of Intersection
      for l =1:1:length(g)
        width = 10;
        height = 5;
        xCenter = g(l,1);
        yCenter = g(l,2);
        xLeft = xCenter - width/2;
        yBottom = yCenter - height/2;
        rectangle('Position', [xLeft, yBottom, width, height], 'EdgeColor', 'b', 'FaceColor', 'r', 'LineWidth', 4);
        grid on;
      end
   hold off; 
   imagePoints(:,:,count)=g; % Feature points across all frames   
end    

%% Camera Parameters

  worldPoints=[0,23.78;1.37,23.78;9.6,23.78;10.97,23.78;0,0;1.37,0;9.6,0;10.97,0];
  params = estimateCameraParameters(imagePoints,worldPoints,'ImageSize', size(frameGray1));
   showExtrinsics(params);
%% Camera Parameters

    k=params.IntrinsicMatrix; % Intrinsic parameter from the estimateCameraParameter function
    hname='C:\UT Files\Image Processing and Computer Vision\Final Project\tennis.avi';
    vidReader2=VideoReader(hname);
    nFrames = vidReader2.NumberOfFrames;
     edgeplay2=VideoWriter('final_tennis.avi');
       open(edgeplay2);
    for iFrame1=1:1:nFrames
        frame1=read(vidReader2,iFrame1);
        r=params.RotationMatrices(:,:,iFrame1);
        t=params.TranslationVectors(iFrame1,:,:);
        point1=[13;14;0;1]; % world coordinates of the corners of advertisement
        point2=[11;14;0;1];
        point3=[11;18;0;1];
        point4=[13;18;0;1];
        mat1 = [r(1,1),r(1,2),r(1,3);r(2,1),r(2,2),r(2,3);r(3,1),r(3,2),r(3,3);t(1,1),t(1,2),t(1,3)];
        pixel1 = point1' * mat1 *k; % Calculating with Intrinsic and Extrinsic Parameters
        pixel1 = [pixel1(1)/pixel1(3);pixel1(2)/pixel1(3)];
        pixel1 = round(pixel1');
        pixel2 = point2' * mat1 *k;
        pixel2 = [pixel2(1)/pixel2(3);pixel2(2)/pixel2(3)];
        pixel2 = round(pixel2');
        pixel3 = point3' * mat1 *k;
        pixel3 = [pixel3(1)/pixel3(3);pixel3(2)/pixel3(3)];
        pixel3 = round(pixel3');
        pixel4 = point4' * mat1 *k;
        pixel4 = [pixel4(1)/pixel4(3);pixel4(2)/pixel4(3)];
        pixel4 = round(pixel4');
        p_bound =[pixel1;pixel2;pixel3;pixel4];
        ad = imread('ut.png'); % Reading the Image
        rpoints=[1,1;1,size(ad,1);size(ad,2),size(ad,1);size(ad,2),1]; % Dimensions of the ad
        h = estimateGeometricTransform(rpoints, p_bound,'projective'); % Geometric Transform
        adv_h = imwarp(ad,h); % Warping the ad to transform
        mask = imwarp(ones(size(ad)),h); % Create a mask with size of ad and warp with transform       
        mask = mask ~= 0;
        xposition = round(min(p_bound(:,1))); % Position of x corner
        yposition = round(min(p_bound(:,2))); % Position of y corner
        t = zeros(size(frame1),'logical'); % mask of the original frame
        mask_org = t(yposition:(yposition+size(adv_h,1)-1),xposition:(xposition+size(adv_h,2)-1),:); 
        mask_org(mask) = 1;
        t(yposition:(yposition+size(adv_h,1)-1),xposition:(xposition+size(adv_h,2)-1),:) = mask_org;
        frame2 = frame1;
        frame2(t)=adv_h(mask);
        filt = fspecial('motion');
        frame2 = imfilter(frame2,filt,'replicate');
        imshow(frame2);
        pause(0.1);
        writeVideo(edgeplay2,frame2);
    end
close (edgeplay2);
   
   
   

   
