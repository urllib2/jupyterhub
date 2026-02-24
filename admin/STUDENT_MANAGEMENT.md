# RosForge Student Management

## Overview

Students are managed via a SQLite database at:
```
/home/sami/ros2-teaching/data/jupyterhub_data/students.db
```

All CLI commands must be run from the `admin/` folder with `sudo`:
```bash
cd /home/sami/ros2-teaching/admin
sudo python3 student_cli.py [command]
```

---

## Commands

### Add a student

A student can log in via **GitHub**, **Google**, or **both**.

**GitHub only:**
```bash
sudo python3 student_cli.py add FirstName LastName --github github_username --payment cash --amount 890
```

**Gmail only:**
```bash
sudo python3 student_cli.py add FirstName LastName --email student@gmail.com --payment cash --amount 890
```

**Both (recommended):**
```bash
sudo python3 student_cli.py add FirstName LastName --github github_username --email student@gmail.com --payment cash --amount 890
```

Payment options: `cash`, `bank`, `transfer`, `other`

---

### List students
```bash
sudo python3 student_cli.py list
sudo python3 student_cli.py list --status active
sudo python3 student_cli.py list --status inactive
```

---

### Remove a student
```bash
sudo python3 student_cli.py remove github_username
# or
sudo python3 student_cli.py remove student@gmail.com
```

This marks the student as inactive — they can no longer log in but their record is kept.

---

### Statistics
```bash
sudo python3 student_cli.py stats
```

---

## How authentication works

- Student logs in with GitHub → system checks `github_username` in database
- Student logs in with Google → system checks `email` in database
- If found and active → access granted
- If not found → 403 Forbidden

Admin (`github:urllib2`) always has access regardless of the database.

---

## Installment payments

For students paying in 3 installments (300 TND/month), add them at first payment:
```bash
sudo python3 student_cli.py add FirstName LastName --email student@gmail.com --payment bank --amount 300 --notes "installment 1/3"
```

---

## Cohort limit

Maximum **10 students**. Check current count:
```bash
sudo python3 student_cli.py stats
```
