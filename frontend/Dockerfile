# Use the official Nginx image
FROM nginx:alpine

# Remove the default nginx index file
RUN rm /usr/share/nginx/html/*

# Copy your index.html into the Nginx HTML directory
COPY index.html /usr/share/nginx/html/index.html

# Expose the port nginx is listening on
EXPOSE 80
