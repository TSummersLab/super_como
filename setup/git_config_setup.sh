#!/bin/bash

# Script sets up the username and email for git config --global to upload to git



#ask for username and email
printf "Please enter a username for git config: "
read username
printf "Please enter an email for git config: "
read email

git config --global user.name "$username"
git config --global user.email "$email"

printf "git config global username and email are setup complete!\n"
printf "\tUsername: $username\n"
printf "\tEmail: $email\n"
